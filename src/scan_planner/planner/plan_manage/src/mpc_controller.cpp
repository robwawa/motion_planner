#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

#include "acados_solver_scan_planar_mpc.h"
#include "bspline_opt/uniform_bspline.h"
#include "scan_planner/Bspline.h"

namespace
{
using scan_planner::UniformBspline;

constexpr int kHorizonSteps = SCAN_PLANAR_MPC_N;
constexpr double kSampleTime = 0.08;
constexpr double kDefaultSolverTimeout = 0.008;
constexpr double kMinSolverTimeout = 0.001;
constexpr double kMaxSolverTimeout = 0.020;

ros::Publisher cmd_pub;
ros::Publisher frozen_pub;
ros::Subscriber bspline_sub;
ros::Subscriber odom_sub;
ros::Timer timer;

scan_planar_mpc_solver_capsule *solver = nullptr;
std::vector<UniformBspline> traj;

bool have_traj = false;
bool have_odom = false;
bool have_final_yaw = false;
bool new_trajectory = false;
bool invalid_input = false;

Eigen::Vector3d position = Eigen::Vector3d::Zero();
Eigen::Vector3d last_u = Eigen::Vector3d::Zero();

double yaw = 0.0;
double terminal_yaw = 0.0;
double duration = 0.0;
double exec_time = 0.0;
ros::Time last_update;

double time_forward = 0.8;
double heading_threshold = 0.8;
double finish_dist = 0.15;
double finish_yaw_tolerance = 0.15;
double max_vx = 0.8;
double max_vy = 0.35;
double max_vyaw = 1.0;
double q_pos = 8.0;
double q_yaw = 1.5;
double r_velocity = 0.08;
double r_rate = 0.4;
double solver_timeout = kDefaultSolverTimeout;
int solver_iterations = 8;
std::string body_pose_topic;

double normalizeAngle(double angle)
{
  if (!std::isfinite(angle))
    return std::numeric_limits<double>::quiet_NaN();

  return std::remainder(angle, 2.0 * M_PI);
}

double unwrapNear(double angle, double anchor)
{
  return anchor + normalizeAngle(angle - anchor);
}

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

bool finiteVector(const Eigen::Vector3d &value)
{
  return value.allFinite();
}

void publishCommand(bool frozen, double vx, double vy, double wz)
{
  std_msgs::Bool frozen_msg;
  frozen_msg.data = frozen;
  frozen_pub.publish(frozen_msg);

  geometry_msgs::Twist command;
  command.linear.x = vx;
  command.linear.y = vy;
  command.angular.z = wz;
  cmd_pub.publish(command);
}

void resetSolverAndStop(double previous_exec_time)
{
  exec_time = previous_exec_time;
  last_u.setZero();
  new_trajectory = true;
  scan_planar_mpc_acados_reset(solver, 1);
  publishCommand(true, 0.0, 0.0, 0.0);
}

bool setReference(int stage, const Eigen::Vector3d &reference, const Eigen::Vector3d &control_reference,
                  const Eigen::Vector3d &previous_control)
{
  double yref[9] = {reference.x(),
                    reference.y(),
                    reference.z(),
                    control_reference.x(),
                    control_reference.y(),
                    control_reference.z(),
                    0.0,
                    0.0,
                    0.0};
  double parameters[3] = {previous_control.x(), previous_control.y(), previous_control.z()};

  return ocp_nlp_cost_model_set(solver->nlp_config, solver->nlp_dims, solver->nlp_in, stage, "yref", yref) ==
             0 &&
         scan_planar_mpc_acados_update_params(solver, stage, parameters, 3) == 0;
}

void configureSolver()
{
  double stage_weights[81] = {0.0};
  double terminal_weights[9] = {0.0};

  stage_weights[0] = q_pos;
  stage_weights[10] = q_pos;
  stage_weights[20] = q_yaw;
  stage_weights[30] = r_velocity;
  stage_weights[40] = r_velocity;
  stage_weights[50] = r_velocity;
  stage_weights[60] = r_rate;
  stage_weights[70] = r_rate;
  stage_weights[80] = r_rate;

  terminal_weights[0] = 1.5 * q_pos;
  terminal_weights[4] = 1.5 * q_pos;
  terminal_weights[8] = 1.5 * q_yaw;

  double lower_control_bounds[3] = {-max_vx, -max_vy, -max_vyaw};
  double upper_control_bounds[3] = {max_vx, max_vy, max_vyaw};

  for (int stage = 0; stage < kHorizonSteps; ++stage)
  {
    ocp_nlp_cost_model_set(solver->nlp_config, solver->nlp_dims, solver->nlp_in, stage, "W", stage_weights);
    ocp_nlp_constraints_model_set(solver->nlp_config, solver->nlp_dims, solver->nlp_in, solver->nlp_out,
                                  stage, "lbu", lower_control_bounds);
    ocp_nlp_constraints_model_set(solver->nlp_config, solver->nlp_dims, solver->nlp_in, solver->nlp_out,
                                  stage, "ubu", upper_control_bounds);
  }

  ocp_nlp_cost_model_set(solver->nlp_config, solver->nlp_dims, solver->nlp_in, kHorizonSteps, "W",
                         terminal_weights);
  ocp_nlp_solver_opts_set(solver->nlp_config, solver->nlp_opts, "max_iter", &solver_iterations);
  ocp_nlp_solver_opts_set(solver->nlp_config, solver->nlp_opts, "timeout_max_time", &solver_timeout);

  double tolerance = 1e-3;
  ocp_nlp_solver_opts_set(solver->nlp_config, solver->nlp_opts, "tol_stat", &tolerance);
  ocp_nlp_solver_opts_set(solver->nlp_config, solver->nlp_opts, "tol_eq", &tolerance);
  ocp_nlp_solver_opts_set(solver->nlp_config, solver->nlp_opts, "tol_ineq", &tolerance);
  ocp_nlp_solver_opts_set(solver->nlp_config, solver->nlp_opts, "tol_comp", &tolerance);

  bool evaluate_at_max_iter = true;
  ocp_nlp_solver_opts_set(solver->nlp_config, solver->nlp_opts, "eval_residual_at_max_iter",
                          &evaluate_at_max_iter);
}

void loadParams(const ros::NodeHandle &private_node)
{
  ros::param::param<std::string>("/body_pose_topic", body_pose_topic, "/quad_0/body_pose");

  private_node.param("time_forward", time_forward, 0.8);
  private_node.param("heading_error_threshold", heading_threshold, 0.8);
  private_node.param("finish_dist", finish_dist, 0.15);
  private_node.param("finish_yaw_tolerance", finish_yaw_tolerance, 0.15);
  private_node.param("max_vx", max_vx, 0.8);
  private_node.param("max_vy", max_vy, 0.35);
  private_node.param("max_vyaw", max_vyaw, 1.0);
  private_node.param("q_pos", q_pos, 8.0);
  private_node.param("q_yaw", q_yaw, 1.5);
  private_node.param("r_velocity", r_velocity, 0.08);
  private_node.param("r_rate", r_rate, 0.4);
  private_node.param("solver_iterations", solver_iterations, 12);
  private_node.param("solver_timeout", solver_timeout, kDefaultSolverTimeout);

  if (!std::isfinite(solver_timeout) || solver_timeout < kMinSolverTimeout ||
      solver_timeout > kMaxSolverTimeout)
  {
    ROS_WARN("[mpc_controller] invalid solver_timeout=%.6f s; using default %.3f s", solver_timeout,
             kDefaultSolverTimeout);
    solver_timeout = kDefaultSolverTimeout;
  }

  max_vyaw = std::min(1.0, max_vyaw);
  solver_iterations = std::max(1, std::min(20, solver_iterations));
}

void bsplineCallback(const scan_planner::BsplineConstPtr &msg)
{
  if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
  {
    ROS_WARN("[mpc_controller] ignoring invalid B-spline");
    return;
  }

  Eigen::MatrixXd control_points(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    control_points(0, i) = msg->pos_pts[i].x;
    control_points(1, i) = msg->pos_pts[i].y;
    control_points(2, i) = msg->pos_pts[i].z;

    if (!control_points.col(i).allFinite())
    {
      ROS_WARN("[mpc_controller] ignoring B-spline with non-finite control point");
      return;
    }
  }

  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
    if (!std::isfinite(knots(i)))
    {
      ROS_WARN("[mpc_controller] ignoring B-spline with non-finite knot");
      return;
    }
  }

  UniformBspline position_traj(control_points, msg->order, 0.1);
  position_traj.setKnot(knots);

  const double next_duration = position_traj.getTimeSum();
  const bool next_have_final_yaw = !msg->yaw_pts.empty();
  const double next_terminal_yaw = next_have_final_yaw ? msg->yaw_pts.back() : 0.0;

  if (!std::isfinite(next_duration) || (next_have_final_yaw && !std::isfinite(next_terminal_yaw)))
  {
    ROS_WARN("[mpc_controller] ignoring B-spline with non-finite duration or yaw");
    return;
  }

  traj.clear();
  traj.push_back(position_traj);
  traj.push_back(traj[0].getDerivative());
  duration = next_duration;
  have_final_yaw = next_have_final_yaw;
  terminal_yaw = next_terminal_yaw;
  exec_time = 0.0;
  last_update = ros::Time::now();
  have_traj = true;
  new_trajectory = true;
  invalid_input = false;
  last_u.setZero();

  ROS_INFO("[mpc_controller] received traj_id=%ld duration=%.3f", static_cast<long>(msg->traj_id), duration);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  const Eigen::Vector3d next_position(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                      msg->pose.pose.position.z);
  const double next_yaw = tf::getYaw(msg->pose.pose.orientation);

  if (!finiteVector(next_position) || !std::isfinite(next_yaw))
  {
    have_odom = false;
    invalid_input = true;
    ROS_ERROR_THROTTLE(1.0, "[mpc_controller] rejecting non-finite odometry");
    return;
  }

  position = next_position;
  yaw = next_yaw;
  have_odom = true;
  invalid_input = false;
}

void timerCallback(const ros::TimerEvent &)
{
  if (!have_traj || !have_odom)
  {
    publishCommand(invalid_input, 0.0, 0.0, 0.0);
    return;
  }

  const ros::Time now = ros::Time::now();
  double dt = (now - last_update).toSec();
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  last_update = now;
  const double previous_exec_time = exec_time;
  double trajectory_time = std::min(duration, exec_time);

  const Eigen::Vector3d current_reference = traj[0].evaluateDeBoorT(trajectory_time);
  Eigen::Vector3d look_direction =
      traj[0].evaluateDeBoorT(std::min(duration, trajectory_time + time_forward)) - current_reference;
  if (look_direction.head<2>().squaredNorm() <= 1e-4)
    look_direction = traj[1].evaluateDeBoorT(trajectory_time);

  double desired_yaw = look_direction.head<2>().squaredNorm() > 1e-4
                           ? std::atan2(look_direction.y(), look_direction.x())
                           : yaw;
  if (have_final_yaw && trajectory_time >= duration - 1e-3)
    desired_yaw = terminal_yaw;

  const double yaw_error = normalizeAngle(desired_yaw - yaw);
  if (std::abs(yaw_error) > heading_threshold)
  {
    publishCommand(true, 0.0, 0.0, clamp(1.5 * yaw_error, -max_vyaw, max_vyaw));
    return;
  }

  exec_time = std::min(duration, exec_time + dt);
  trajectory_time = exec_time;

  std::vector<Eigen::Vector3d> references(kHorizonSteps + 1);
  std::vector<Eigen::Vector3d> control_references(kHorizonSteps, Eigen::Vector3d::Zero());
  double yaw_anchor = yaw;

  for (int stage = 0; stage <= kHorizonSteps; ++stage)
  {
    const double stage_time = std::min(duration, trajectory_time + stage * kSampleTime);
    const Eigen::Vector3d stage_position = traj[0].evaluateDeBoorT(stage_time);
    Eigen::Vector3d stage_direction =
        traj[0].evaluateDeBoorT(std::min(duration, stage_time + time_forward)) - stage_position;

    if (stage_direction.head<2>().squaredNorm() <= 1e-4)
      stage_direction = traj[1].evaluateDeBoorT(stage_time);

    double raw_yaw = stage_direction.head<2>().squaredNorm() > 1e-4
                         ? std::atan2(stage_direction.y(), stage_direction.x())
                         : yaw_anchor;
    if (have_final_yaw && stage_time >= duration - 1e-3)
      raw_yaw = terminal_yaw;

    references[stage] = Eigen::Vector3d(stage_position.x(), stage_position.y(), unwrapNear(raw_yaw, yaw_anchor));
    yaw_anchor = references[stage].z();
  }

  for (int stage = 0; stage < kHorizonSteps; ++stage)
  {
    const double stage_time = std::min(duration, trajectory_time + stage * kSampleTime);
    const Eigen::Vector3d world_velocity = traj[1].evaluateDeBoorT(stage_time);
    const double cos_yaw = std::cos(references[stage].z());
    const double sin_yaw = std::sin(references[stage].z());

    control_references[stage].x() = cos_yaw * world_velocity.x() + sin_yaw * world_velocity.y();
    control_references[stage].y() = -sin_yaw * world_velocity.x() + cos_yaw * world_velocity.y();
    control_references[stage].z() =
        normalizeAngle(references[stage + 1].z() - references[stage].z()) / kSampleTime;

    control_references[stage].x() = clamp(control_references[stage].x(), -max_vx, max_vx);
    control_references[stage].y() = clamp(control_references[stage].y(), -max_vy, max_vy);
    control_references[stage].z() = clamp(control_references[stage].z(), -max_vyaw, max_vyaw);
  }

  for (int stage = 0; stage <= kHorizonSteps; ++stage)
  {
    if (!finiteVector(references[stage]) ||
        (stage < kHorizonSteps && !finiteVector(control_references[stage])))
    {
      ROS_ERROR_THROTTLE(1.0, "[mpc_controller] non-finite MPC reference; resetting solver");
      resetSolverAndStop(previous_exec_time);
      return;
    }
  }

  std::vector<Eigen::Vector3d> warm_controls(kHorizonSteps, Eigen::Vector3d::Zero());
  for (int stage = 0; stage < kHorizonSteps; ++stage)
  {
    if (new_trajectory)
    {
      warm_controls[stage] = control_references[stage];
    }
    else
    {
      double old_control[3] = {0.0, 0.0, 0.0};
      ocp_nlp_out_get(solver->nlp_config, solver->nlp_dims, solver->nlp_out,
                      std::min(stage + 1, kHorizonSteps - 1), "u", old_control);
      warm_controls[stage] << old_control[0], old_control[1], old_control[2];
      if (!finiteVector(warm_controls[stage]))
        warm_controls[stage] = control_references[stage];
    }

    double control_guess[3] = {warm_controls[stage].x(), warm_controls[stage].y(), warm_controls[stage].z()};
    double state_guess[3] = {references[stage].x(), references[stage].y(), references[stage].z()};
    ocp_nlp_out_set(solver->nlp_config, solver->nlp_dims, solver->nlp_out, solver->nlp_in, stage, "u",
                    control_guess);
    ocp_nlp_out_set(solver->nlp_config, solver->nlp_dims, solver->nlp_out, solver->nlp_in, stage, "x",
                    state_guess);
  }

  double terminal_state[3] = {references[kHorizonSteps].x(), references[kHorizonSteps].y(),
                              references[kHorizonSteps].z()};
  ocp_nlp_out_set(solver->nlp_config, solver->nlp_dims, solver->nlp_out, solver->nlp_in, kHorizonSteps, "x",
                  terminal_state);
  new_trajectory = false;

  for (int stage = 0; stage < kHorizonSteps; ++stage)
  {
    const Eigen::Vector3d &previous_control = stage == 0 ? last_u : warm_controls[stage - 1];
    if (!setReference(stage, references[stage], control_references[stage], previous_control))
    {
      publishCommand(true, 0.0, 0.0, 0.0);
      return;
    }
  }

  if (!setReference(kHorizonSteps, references[kHorizonSteps], Eigen::Vector3d::Zero(),
                    control_references[kHorizonSteps - 1]))
  {
    publishCommand(true, 0.0, 0.0, 0.0);
    return;
  }

  double initial_state[3] = {position.x(), position.y(), yaw};
  const int lower_bound_status =
      ocp_nlp_constraints_model_set(solver->nlp_config, solver->nlp_dims, solver->nlp_in, solver->nlp_out, 0,
                                    "lbx", initial_state);
  const int upper_bound_status =
      ocp_nlp_constraints_model_set(solver->nlp_config, solver->nlp_dims, solver->nlp_in, solver->nlp_out, 0,
                                    "ubx", initial_state);
  if (lower_bound_status != 0 || upper_bound_status != 0)
  {
    publishCommand(true, 0.0, 0.0, 0.0);
    return;
  }

  const auto solve_start = std::chrono::steady_clock::now();
  const int status = scan_planar_mpc_acados_solve(solver);
  const double wall_time =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - solve_start).count();

  double solve_time = 0.0;
  int sqp_iterations = 0;
  ocp_nlp_get(solver->nlp_solver, "time_tot", &solve_time);
  ocp_nlp_get(solver->nlp_solver, "sqp_iter", &sqp_iterations);

  const bool timed_out = status == ACADOS_TIMEOUT || wall_time > solver_timeout;
  if (timed_out)
  {
    ROS_ERROR_THROTTLE(1.0,
                       "[mpc_controller] acados solve timeout: status=%d, acados=%.3f ms, "
                       "wall=%.3f ms, limit=%.3f ms",
                       status, solve_time * 1000.0, wall_time * 1000.0, solver_timeout * 1000.0);
    resetSolverAndStop(previous_exec_time);
    return;
  }

  if (status != ACADOS_SUCCESS && status != ACADOS_MAXITER)
  {
    ROS_ERROR_THROTTLE(1.0, "[mpc_controller] acados solve failed: %d", status);
    resetSolverAndStop(previous_exec_time);
    return;
  }

  double control[3] = {0.0, 0.0, 0.0};
  ocp_nlp_out_get(solver->nlp_config, solver->nlp_dims, solver->nlp_out, 0, "u", control);
  if (!std::isfinite(control[0]) || !std::isfinite(control[1]) || !std::isfinite(control[2]))
  {
    ROS_ERROR_THROTTLE(1.0, "[mpc_controller] acados returned non-finite control");
    resetSolverAndStop(previous_exec_time);
    return;
  }

  if (status == ACADOS_MAXITER)
  {
    ROS_WARN_THROTTLE(1.0, "[mpc_controller] acados reached max iterations; using current solution");
  }

  ROS_DEBUG_THROTTLE(1.0, "[mpc_controller] solve %.3f ms (wall %.3f ms), SQP iterations %d",
                     solve_time * 1000.0, wall_time * 1000.0, sqp_iterations);

  const Eigen::Vector3d terminal_reference = references[kHorizonSteps];
  const double terminal_position_error = (position.head<2>() - terminal_reference.head<2>()).norm();
  const double terminal_yaw_error = std::abs(normalizeAngle(yaw - terminal_reference.z()));
  if (exec_time >= duration && terminal_position_error < finish_dist &&
      terminal_yaw_error < finish_yaw_tolerance)
  {
    control[0] = 0.0;
    control[1] = 0.0;
    control[2] = 0.0;
  }

  last_u << control[0], control[1], control[2];
  publishCommand(false, clamp(control[0], -max_vx, max_vx), clamp(control[1], -max_vy, max_vy),
                 clamp(control[2], -max_vyaw, max_vyaw));
}

void shutdownController()
{
  timer.stop();
  bspline_sub.shutdown();
  odom_sub.shutdown();
  cmd_pub.shutdown();
  frozen_pub.shutdown();
  scan_planar_mpc_acados_free(solver);
  scan_planar_mpc_acados_free_capsule(solver);
  solver = nullptr;
}
} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_controller");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  loadParams(private_node);

  solver = scan_planar_mpc_acados_create_capsule();
  if (!solver || scan_planar_mpc_acados_create(solver) != 0)
  {
    ROS_FATAL("[mpc_controller] failed to create acados solver");
    return 1;
  }
  configureSolver();

  cmd_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 20);
  frozen_pub = node.advertise<std_msgs::Bool>("planning/go2_execution_frozen", 10);
  bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  odom_sub = node.subscribe(body_pose_topic, 20, odomCallback, ros::TransportHints().tcpNoDelay());
  timer = node.createTimer(ros::Duration(0.01), timerCallback);

  ROS_WARN("[mpc_controller] acados solver ready (N=%d, dt=%.3f, timeout=%.3f ms)", kHorizonSteps,
           kSampleTime, solver_timeout * 1000.0);
  ros::spin();

  shutdownController();
  return 0;
}
