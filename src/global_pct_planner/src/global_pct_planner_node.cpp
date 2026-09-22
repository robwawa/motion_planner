#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <boost/bind/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>

#include <global_pct_planner/PlanPath3DAction.h>
#include <global_pct_planner/PctTerrainMap.h>
#include <motion_planner_log/logging.h>
#include "global_pct_planner/dynamic_obstacle_layer.h"
#include "global_pct_planner/planner_core.h"
#include "global_pct_planner/tomogram_io.h"

namespace global_pct_planner {
namespace {

bool finitePose(const geometry_msgs::Pose& pose) {
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

std::size_t datatypeSize(uint8_t datatype) {
  switch (datatype) {
    case sensor_msgs::PointField::INT8:
    case sensor_msgs::PointField::UINT8:
      return 1;
    case sensor_msgs::PointField::INT16:
    case sensor_msgs::PointField::UINT16:
      return 2;
    case sensor_msgs::PointField::INT32:
    case sensor_msgs::PointField::UINT32:
    case sensor_msgs::PointField::FLOAT32:
      return 4;
    case sensor_msgs::PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

float readField(const uint8_t* address, uint8_t datatype, bool big_endian) {
  const std::size_t size = datatypeSize(datatype);
  if (size == 0) throw std::runtime_error("unsupported PointCloud2 field type");
  uint8_t bytes[8] = {};
  std::memcpy(bytes, address, size);
  if (big_endian) std::reverse(bytes, bytes + size);
  switch (datatype) {
    case sensor_msgs::PointField::INT8:
      return static_cast<float>(*reinterpret_cast<int8_t*>(bytes));
    case sensor_msgs::PointField::UINT8:
      return static_cast<float>(*reinterpret_cast<uint8_t*>(bytes));
    case sensor_msgs::PointField::INT16: {
      int16_t value;
      std::memcpy(&value, bytes, sizeof(value));
      return static_cast<float>(value);
    }
    case sensor_msgs::PointField::UINT16: {
      uint16_t value;
      std::memcpy(&value, bytes, sizeof(value));
      return static_cast<float>(value);
    }
    case sensor_msgs::PointField::INT32: {
      int32_t value;
      std::memcpy(&value, bytes, sizeof(value));
      return static_cast<float>(value);
    }
    case sensor_msgs::PointField::UINT32: {
      uint32_t value;
      std::memcpy(&value, bytes, sizeof(value));
      return static_cast<float>(value);
    }
    case sensor_msgs::PointField::FLOAT32: {
      float value;
      std::memcpy(&value, bytes, sizeof(value));
      return value;
    }
    case sensor_msgs::PointField::FLOAT64: {
      double value;
      std::memcpy(&value, bytes, sizeof(value));
      return static_cast<float>(value);
    }
    default:
      throw std::runtime_error("unsupported PointCloud2 field type");
  }
}

std::vector<PointXYZ> parseCloud(const sensor_msgs::PointCloud2& message,
                                 std::size_t max_points) {
  const sensor_msgs::PointField* x_field = nullptr;
  const sensor_msgs::PointField* y_field = nullptr;
  const sensor_msgs::PointField* z_field = nullptr;
  for (const sensor_msgs::PointField& field : message.fields) {
    if (field.name == "x") x_field = &field;
    if (field.name == "y") y_field = &field;
    if (field.name == "z") z_field = &field;
  }
  if (!x_field || !y_field || !z_field) {
    throw std::runtime_error("PointCloud2 has no x/y/z fields");
  }
  if (message.point_step == 0 || message.width == 0 || message.height == 0) {
    return {};
  }
  const std::size_t count = static_cast<std::size_t>(message.width) *
                            static_cast<std::size_t>(message.height);
  const std::size_t row_step = message.row_step != 0
                                   ? message.row_step
                                   : message.width * message.point_step;
  const std::size_t required_bytes =
      static_cast<std::size_t>(message.height - 1) * row_step +
      static_cast<std::size_t>(message.width) * message.point_step;
  if (required_bytes > message.data.size()) {
    throw std::runtime_error("PointCloud2 data is shorter than width*height");
  }
  const std::size_t stride =
      max_points > 0 && count > max_points
          ? static_cast<std::size_t>(std::ceil(count / static_cast<double>(max_points)))
          : 1;
  const std::size_t x_size = datatypeSize(x_field->datatype);
  const std::size_t y_size = datatypeSize(y_field->datatype);
  const std::size_t z_size = datatypeSize(z_field->datatype);
  if (x_size == 0 || y_size == 0 || z_size == 0 ||
      x_field->offset + x_size > message.point_step ||
      y_field->offset + y_size > message.point_step ||
      z_field->offset + z_size > message.point_step) {
    throw std::runtime_error("PointCloud2 field exceeds point_step");
  }
  std::vector<PointXYZ> points;
  points.reserve(std::min(count, max_points > 0 ? max_points : count));
  for (std::size_t i = 0; i < count; i += stride) {
    const std::size_t row = i / message.width;
    const std::size_t col = i % message.width;
    const uint8_t* base = message.data.data() + row * row_step +
                          col * message.point_step;
    const PointXYZ point{readField(base + x_field->offset, x_field->datatype,
                                   message.is_bigendian),
                         readField(base + y_field->offset, y_field->datatype,
                                   message.is_bigendian),
                         readField(base + z_field->offset, z_field->datatype,
                                   message.is_bigendian)};
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z)) {
      points.push_back(point);
    }
  }
  return points;
}

sensor_msgs::PointCloud2 makeDebugCloud(
    const std::vector<PointXYZ>& points, const std::string& frame) {
  sensor_msgs::PointCloud2 message;
  message.header.frame_id = frame;
  message.header.stamp = ros::Time::now();
  message.height = 1;
  message.width = static_cast<uint32_t>(points.size());
  message.is_dense = false;
  message.fields.resize(3);
  message.fields[0].name = "x";
  message.fields[0].offset = 0;
  message.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  message.fields[0].count = 1;
  message.fields[1] = message.fields[0];
  message.fields[1].name = "y";
  message.fields[1].offset = 4;
  message.fields[2] = message.fields[0];
  message.fields[2].name = "z";
  message.fields[2].offset = 8;
  message.point_step = 12;
  message.row_step = message.point_step * message.width;
  message.data.resize(message.row_step);
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::memcpy(message.data.data() + i * 12, &points[i].x, sizeof(float));
    std::memcpy(message.data.data() + i * 12 + 4, &points[i].y, sizeof(float));
    std::memcpy(message.data.data() + i * 12 + 8, &points[i].z, sizeof(float));
  }
  return message;
}

}  // namespace

class GlobalPctPlannerNode {
 public:
  using ActionServer = actionlib::SimpleActionServer<PlanPath3DAction>;

  GlobalPctPlannerNode()
      : private_handle_("~"),
        threshold_(readParam("/pct/traversability/cost_threshold", 49.0f)),
        core_(threshold_),
        action_server_(private_handle_, "/pct/plan_path",
                       boost::bind(&GlobalPctPlannerNode::execute, this,
                                   boost::placeholders::_1),
                       false) {
    private_handle_.param<std::string>("navigation_frame", navigation_frame_,
                                       "map");
    private_handle_.param("body_height", body_height_, 0.4f);
    private_handle_.param("optimize_path", optimize_path_, true);
    private_handle_.param("endpoint_snap_radius", endpoint_snap_radius_, 1.5f);
    private_handle_.param("wait_timeout", wait_timeout_, 300.0);
    private_handle_.param("max_heading_rate", max_heading_rate_, 10.0f);
    private_handle_.param("use_quintic", use_quintic_, true);
    private_handle_.param<std::string>("tomogram_name", tomogram_name_,
                                       "building2_9");
    private_handle_.param<std::string>("tomogram_dir", tomogram_dir_, "");
    if (tomogram_dir_.empty()) {
      throw std::runtime_error(
          "~tomogram_dir is empty. Set the tomogram directory in the launch "
          "file, for example src/global_pct_planner/rsc/tomogram.");
    }
    if (!tomogram_dir_.empty() && tomogram_dir_.back() != '/') tomogram_dir_ += '/';
    loadTomogram();

    global_path_publisher_ =
        nh_.advertise<nav_msgs::Path>("/pct/global_path", 1, true);
    dynamic_ok_publisher_ = nh_.advertise<std_msgs::Bool>(
        "/pct/dynamic_layer_ok", 1, true);
    dynamic_version_publisher_ = nh_.advertise<std_msgs::UInt64>(
        "/pct/dynamic_version", 1, true);
    dynamic_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/pct/dynamic_cloud", 1);
    dynamic_cost_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/pct/dynamic_costmap", 1);

    private_handle_.param("dynamic_replan/enabled", dynamic_enabled_, false);
    if (dynamic_enabled_) {
      try {
        initDynamicLayer();
      } catch (const std::exception& exception) {
        MOTION_PLANNER_LOG_ERROR_STREAM("dynamic replan disabled: "
                                        << exception.what());
        dynamic_enabled_ = false;
        publishDynamicOk(false);
      }
    } else {
      publishDynamicOk(true);
    }
    endpoint_snap_radius_cells_ = std::max(
        0, static_cast<int>(std::ceil(endpoint_snap_radius_ / map_.resolution)));
    action_server_.start();
    MOTION_PLANNER_LOG_INFO(
        "Ready: tomogram=%s frame=%s optimize_path=%s dynamic_replan=%s "
        "snap_radius=%.2f wait_timeout=%.1f",
        tomogram_name_.c_str(), navigation_frame_.c_str(),
        optimize_path_ ? "true" : "false", dynamic_enabled_ ? "true" : "false",
        endpoint_snap_radius_, wait_timeout_);
  }

 private:
  template <typename T>
  static T readParam(const std::string& name, const T& fallback) {
    T value;
    ros::param::param(name, value, fallback);
    return value;
  }

  void loadTomogram() {
    const std::string path = tomogram_dir_ + tomogram_name_ + ".pctm";
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(std::max(0.0, wait_timeout_));
    std::string error;
    while (ros::ok() && !TomogramIO::load(path, map_, &error)) {
      if (ros::WallTime::now() >= deadline) {
        throw std::runtime_error("timed out waiting for tomogram " + path +
                                 ": " + error);
      }
      MOTION_PLANNER_LOG_INFO_THROTTLE(5.0, "Waiting for .pctm map: %s",
                                       path.c_str());
      ros::WallDuration(0.2).sleep();
    }
    if (!ros::ok()) throw std::runtime_error("ROS stopped while loading map");
    core_.load(map_, max_heading_rate_, use_quintic_);
  }

  void initDynamicLayer() {
    const std::string prefix = "dynamic_replan/";
    int lethal = 100;
    double max_memory = 128.0;
    int max_points = 200000;
    private_handle_.param(prefix + "lethal_cost", lethal, 100);
    private_handle_.param(prefix + "max_dynamic_memory_mb", max_memory, 128.0);
    private_handle_.param(prefix + "max_points_per_snapshot", max_points, 200000);
    float terrain_ignore = 0.08f;
    float layer_tolerance = 0.25f;
    float robot_radius = 0.32f;
    float safety_margin = 0.15f;
    float inflation_radius = 0.5f;
    float collision_top_margin = 0.2f;
    private_handle_.param(prefix + "terrain_ignore_height", terrain_ignore, 0.08f);
    private_handle_.param(prefix + "layer_height_tolerance", layer_tolerance, 0.25f);
    private_handle_.param(prefix + "robot_radius", robot_radius, 0.32f);
    private_handle_.param(prefix + "safety_margin", safety_margin, 0.15f);
    private_handle_.param(prefix + "inflation_radius", inflation_radius, 0.5f);
    private_handle_.param(prefix + "collision_top_margin", collision_top_margin, 0.2f);
    dynamic_layer_ = std::make_unique<DynamicObstacleLayer>(
        map_, threshold_, static_cast<uint8_t>(lethal), terrain_ignore,
        body_height_ + collision_top_margin, layer_tolerance, robot_radius,
        safety_margin, inflation_radius, max_points, max_memory);
    dynamic_lethal_cost_ = static_cast<uint8_t>(lethal);
    private_handle_.param(prefix + "max_debug_points", max_debug_points_, 100000);
    private_handle_.param(prefix + "max_snapshot_retries", max_snapshot_retries_, 1);
    private_handle_.param(prefix + "dynamic_source_timeout", dynamic_source_timeout_,
                          0.5);
    if (lethal <= 0 || lethal > 255) {
      throw std::invalid_argument("dynamic lethal_cost must be in [1, 255]");
    }
    max_debug_points_ = std::max(0, max_debug_points_);
    max_snapshot_retries_ = std::max(0, max_snapshot_retries_);
    dynamic_source_timeout_ = std::max(0.0, dynamic_source_timeout_);
    std::string cloud_topic = "/dynamic_perception/dynamic_cloud";
    std::string health_topic = "/dynamic_perception/scan_healthy";
    private_handle_.param(prefix + "cloud_topic", cloud_topic, cloud_topic);
    private_handle_.param(prefix + "health_topic", health_topic, health_topic);
    health_subscriber_ = nh_.subscribe(health_topic, 1,
                                        &GlobalPctPlannerNode::healthCallback,
                                        this);
    ros::SubscribeOptions cloud_options;
    cloud_options.initByFullCallbackType<sensor_msgs::PointCloud2ConstPtr>(
        cloud_topic, 1,
        boost::bind(&GlobalPctPlannerNode::cloudCallback, this,
                    boost::placeholders::_1));
    cloud_options.transport_hints = ros::TransportHints().tcpNoDelay();
    cloud_subscriber_ = nh_.subscribe(cloud_options);
    double update_rate = 10.0;
    double debug_rate = 1.0;
    private_handle_.param(prefix + "update_rate", update_rate, 10.0);
    private_handle_.param(prefix + "debug_publish_rate", debug_rate, 1.0);
    decay_timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, update_rate)),
                                   &GlobalPctPlannerNode::decayCallback, this);
    debug_timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(0.1, debug_rate)),
        &GlobalPctPlannerNode::debugCallback, this);
    publishDynamicOk(false);
    MOTION_PLANNER_LOG_INFO(
        "Dynamic layer enabled: cells=%zu cloud_topic=%s health_topic=%s",
        map_.cellCount(), cloud_topic.c_str(), health_topic.c_str());
  }

  void publishDynamicOk(bool ok) {
    std_msgs::Bool message;
    message.data = ok;
    dynamic_ok_publisher_.publish(message);
  }

  void publishVersion() {
    if (!dynamic_layer_) return;
    std_msgs::UInt64 message;
    message.data = dynamic_layer_->snapshot()->version;
    dynamic_version_publisher_.publish(message);
  }

  void healthCallback(const std_msgs::BoolConstPtr& message) {
    const bool was_healthy = dynamic_source_healthy_;
    dynamic_source_healthy_ = message->data;
    if (dynamic_source_healthy_ && !was_healthy) {
      MOTION_PLANNER_LOG_INFO("Dynamic perception input recovered");
    } else if (!dynamic_source_healthy_ && was_healthy) {
      MOTION_PLANNER_LOG_WARN("Dynamic perception input became unhealthy");
    }
    if (!dynamic_source_healthy_) {
      last_dynamic_update_ = ros::WallTime();
      if (dynamic_layer_ && dynamic_layer_->clear()) publishVersion();
      publishDynamicOk(false);
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    if (!dynamic_source_healthy_ || !dynamic_layer_) return;
    try {
      if (message->header.frame_id != navigation_frame_) {
        throw std::runtime_error("dynamic cloud frame does not match navigation frame");
      }
      const std::vector<PointXYZ> points = parseCloud(
          *message, static_cast<std::size_t>(dynamic_layer_->maxPointsPerSnapshot()));
      dynamic_layer_->replaceSnapshot(points);
      last_dynamic_update_ = ros::WallTime::now();
      publishVersion();
      publishDynamicOk(true);
    } catch (const std::exception& exception) {
      publishDynamicOk(false);
      MOTION_PLANNER_LOG_WARN_THROTTLE(2.0, "Dynamic cloud rejected: %s",
                                       exception.what());
    }
  }

  void decayCallback(const ros::TimerEvent&) {
    if (!dynamic_layer_ || last_dynamic_update_.isZero()) return;
    if ((ros::WallTime::now() - last_dynamic_update_).toSec() >
        dynamic_source_timeout_) {
      last_dynamic_update_ = ros::WallTime();
      dynamic_source_healthy_ = false;
      if (dynamic_layer_->clear()) publishVersion();
      publishDynamicOk(false);
      MOTION_PLANNER_LOG_WARN_THROTTLE(
          2.0, "Dynamic perception timed out; projection cleared");
    }
  }

  void debugCallback(const ros::TimerEvent&) {
    if (!dynamic_layer_) return;
    if (dynamic_cloud_publisher_.getNumSubscribers() > 0) {
      const auto assigned = dynamic_layer_->assignedPoints();
      std::vector<PointXYZ> points;
      points.reserve(assigned.size());
      for (const auto& point : assigned) {
        const auto world = dynamic_layer_->gridToWorld(point.row, point.col);
        points.push_back({world.first, world.second, point.z});
      }
      if (max_debug_points_ > 0 &&
          points.size() > static_cast<std::size_t>(max_debug_points_)) {
        points.resize(static_cast<std::size_t>(max_debug_points_));
      }
      dynamic_cloud_publisher_.publish(makeDebugCloud(points, navigation_frame_));
    }
    if (dynamic_cost_publisher_.getNumSubscribers() > 0) {
      const auto cells = dynamic_layer_->nonzeroCells(max_debug_points_);
      std::vector<PointXYZ> points;
      points.reserve(cells.size());
      for (const auto& cell : cells) {
        const auto world = dynamic_layer_->gridToWorld(cell.row, cell.col);
        points.push_back({world.first, world.second, cell.z});
      }
      dynamic_cost_publisher_.publish(makeDebugCloud(points, navigation_frame_));
    }
  }

  void feedback(const std::string& stage) {
    PlanPath3DFeedback message;
    message.stage = stage;
    action_server_.publishFeedback(message);
  }

  geometry_msgs::PoseStamped snappedPose(
      const geometry_msgs::PoseStamped& source, const SnapResult& position) const {
    geometry_msgs::PoseStamped result = source;
    result.header.frame_id = navigation_frame_;
    result.header.stamp = ros::Time::now();
    result.pose.position.x = position.x;
    result.pose.position.y = position.y;
    result.pose.position.z = position.z;
    return result;
  }

  PlanPath3DResult result(uint8_t status, const std::string& message,
                          const nav_msgs::Path* path,
                          const geometry_msgs::PoseStamped* start,
                          const geometry_msgs::PoseStamped* goal,
                          float start_distance, float goal_distance) const {
    PlanPath3DResult output;
    output.status = status;
    output.message = message;
    if (path) output.path = *path;
    if (start) {
      output.has_snapped_start = true;
      output.snapped_start = *start;
      output.snapped_start_distance = start_distance;
    }
    if (goal) {
      output.has_snapped_goal = true;
      output.snapped_goal = *goal;
      output.snapped_goal_distance = goal_distance;
    }
    return output;
  }

  void fail(uint8_t status, const std::string& message,
            const geometry_msgs::PoseStamped* start = nullptr,
            const geometry_msgs::PoseStamped* goal = nullptr,
            float start_distance = 0.0f, float goal_distance = 0.0f) {
    const PlanPath3DResult output =
        result(status, message, nullptr, start, goal, start_distance,
               goal_distance);
    action_server_.setAborted(output, message);
  }

  nav_msgs::Path makePath(const PlanOutput& plan,
                          const geometry_msgs::PoseStamped& requested_goal) const {
    nav_msgs::Path path;
    path.header.frame_id = navigation_frame_;
    path.header.stamp = ros::Time::now();
    path.poses.reserve(plan.path.size());
    for (std::size_t i = 0; i < plan.path.size(); ++i) {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = plan.path[i].x;
      pose.pose.position.y = plan.path[i].y;
      pose.pose.position.z = plan.path[i].z;
      if (i + 1 < plan.path.size()) {
        const float dx = plan.path[i + 1].x - plan.path[i].x;
        const float dy = plan.path[i + 1].y - plan.path[i].y;
        const double yaw = std::atan2(dy, dx);
        pose.pose.orientation.z = std::sin(yaw * 0.5);
        pose.pose.orientation.w = std::cos(yaw * 0.5);
      } else {
        pose.pose.orientation = requested_goal.pose.orientation;
      }
      path.poses.push_back(pose);
    }
    return path;
  }

  void execute(const PlanPath3DGoalConstPtr& goal) {
    feedback("validating");
    if (goal->start.header.frame_id != navigation_frame_ ||
        goal->goal.header.frame_id != navigation_frame_) {
      fail(PlanPath3DResult::FRAME_MISMATCH,
           "start and goal must be expressed in " + navigation_frame_);
      return;
    }
    if (!finitePose(goal->start.pose) || !finitePose(goal->goal.pose)) {
      fail(PlanPath3DResult::INVALID_REQUEST,
           "poses contain non-finite values");
      return;
    }
    if (action_server_.isPreemptRequested()) {
      action_server_.setPreempted(
          result(PlanPath3DResult::PREEMPTED, "preempted", nullptr, nullptr,
                 nullptr, 0.0f, 0.0f),
          "preempted");
      return;
    }

    SnapResult start;
    SnapResult end;
    PlanOutput planned;
    std::shared_ptr<const DynamicSnapshot> snapshot;
    try {
      const int attempts = dynamic_enabled_
                               ? 1 + std::max(0, max_snapshot_retries_)
                               : 1;
      bool stable = false;
      for (int attempt = 0; attempt < attempts; ++attempt) {
        feedback("snapping_endpoints");
        std::lock_guard<std::mutex> lock(planner_mutex_);
        snapshot = dynamic_enabled_ ? dynamic_layer_->snapshot() : nullptr;
        if (snapshot) core_.setDynamicSnapshot(snapshot);
        else core_.clearDynamicSnapshot();
        start = core_.snapToTraversable(
            {static_cast<float>(goal->start.pose.position.x),
             static_cast<float>(goal->start.pose.position.y),
             static_cast<float>(goal->start.pose.position.z)},
            body_height_, endpoint_snap_radius_cells_);
        end = core_.snapToTraversable(
            {static_cast<float>(goal->goal.pose.position.x),
             static_cast<float>(goal->goal.pose.position.y),
             static_cast<float>(goal->goal.pose.position.z)},
            body_height_, endpoint_snap_radius_cells_);
        if (!start.found || !end.found) {
          fail(PlanPath3DResult::NO_TRAVERSABLE_LAYER,
               "no traversable surface near start or goal");
          return;
        }
        const geometry_msgs::PoseStamped snapped_start =
            snappedPose(goal->start, start);
        const geometry_msgs::PoseStamped snapped_goal = snappedPose(goal->goal, end);
        feedback("planning");
        if (!core_.plan(start, end, body_height_, optimize_path_, planned)) {
          fail(PlanPath3DResult::NO_PATH, "PCT did not find a path",
               &snapped_start, &snapped_goal, start.distance, end.distance);
          return;
        }
        if (action_server_.isPreemptRequested()) {
          action_server_.setPreempted(
              result(PlanPath3DResult::PREEMPTED, "preempted", nullptr,
                     &snapped_start, &snapped_goal, start.distance,
                     end.distance),
              "preempted");
          return;
        }
        const uint64_t current_version =
            dynamic_enabled_ ? dynamic_layer_->snapshot()->version : 0;
        stable = !dynamic_enabled_ || current_version == snapshot->version;
        if (stable) {
          // The legacy action server rejects A* results shorter than two
          // points.  This is relevant when start and goal are adjacent and
          // the native A* result contains only the goal cell.
          if (planned.path.size() < 2) {
            fail(PlanPath3DResult::NO_PATH, "PCT path has fewer than two points",
                 &snapped_start, &snapped_goal, start.distance,
                 end.distance);
            return;
          }
          nav_msgs::Path path = makePath(planned, goal->goal);
          if (!path.poses.empty()) {
            // Keep the legacy wrapper's public-path behavior: only the final
            // pose is replaced by the exact snapped goal.  The first pose is
            // left as returned by A*/GPMP.
            path.poses.back().pose.position.x = end.x;
            path.poses.back().pose.position.y = end.y;
            path.poses.back().pose.position.z = end.z;
          }
          global_path_publisher_.publish(path);
          const std::string message =
              "ok; start snapped " + std::to_string(start.distance) +
              "m, goal snapped " + std::to_string(end.distance) + "m";
          feedback("publishing");
          action_server_.setSucceeded(
              result(PlanPath3DResult::SUCCESS, message, &path, &snapped_start,
                     &snapped_goal, start.distance, end.distance),
              "path found");
          return;
        }
      }
      if (!stable) {
        fail(PlanPath3DResult::NO_PATH,
             "dynamic layer changed during planning; path rejected");
      }
    } catch (const std::exception& exception) {
      MOTION_PLANNER_LOG_ERROR_STREAM("Global PCT planning failed: "
                                      << exception.what());
      fail(PlanPath3DResult::INTERNAL_ERROR, exception.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_handle_;
  std::string navigation_frame_;
  std::string tomogram_name_;
  std::string tomogram_dir_;
  float threshold_{49.0f};
  float body_height_{0.4f};
  float endpoint_snap_radius_{1.5f};
  float max_heading_rate_{10.0f};
  bool optimize_path_{true};
  bool use_quintic_{true};
  double wait_timeout_{300.0};
  int endpoint_snap_radius_cells_{0};
  TomogramData map_;
  PlannerCore core_;
  std::unique_ptr<DynamicObstacleLayer> dynamic_layer_;
  bool dynamic_enabled_{false};
  bool dynamic_source_healthy_{false};
  uint8_t dynamic_lethal_cost_{100};
  int max_debug_points_{100000};
  int max_snapshot_retries_{1};
  double dynamic_source_timeout_{0.5};
  ros::WallTime last_dynamic_update_;
  std::mutex planner_mutex_;

  ros::Publisher global_path_publisher_;
  ros::Publisher dynamic_ok_publisher_;
  ros::Publisher dynamic_version_publisher_;
  ros::Publisher dynamic_cloud_publisher_;
  ros::Publisher dynamic_cost_publisher_;
  ros::Subscriber health_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Timer decay_timer_;
  ros::Timer debug_timer_;
  ActionServer action_server_;
};

}  // namespace global_pct_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_pct_planner");
  motion_planner_log::initialize("global_pct_planner", argv[0]);
  try {
    global_pct_planner::GlobalPctPlannerNode node;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception& exception) {
    MOTION_PLANNER_LOG_FATAL("Failed to start: %s", exception.what());
    return 1;
  }
  return 0;
}
