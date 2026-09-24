#include "navigation_supervisor/reference_path_processor.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace navigation_supervisor {

ReferencePathProcessor::ReferencePathProcessor(std::string navigation_frame,
                                               double max_start_distance)
    : navigation_frame_(std::move(navigation_frame)),
      max_start_distance_(std::max(0.0, max_start_distance)) {}

double ReferencePathProcessor::distance(const geometry_msgs::Point& lhs,
                                        const geometry_msgs::Point& rhs) {
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  const double dz = lhs.z - rhs.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool ReferencePathProcessor::process(
    const nav_msgs::Path& input,
    const geometry_msgs::PoseStamped& request_start,
    nav_msgs::Path& output,
    std::string& error) const {
  output = nav_msgs::Path();
  error.clear();

  if (input.header.frame_id != navigation_frame_ || input.poses.size() < 2) {
    error = "invalid_global_path";
    return false;
  }
  if (request_start.header.frame_id != navigation_frame_) {
    error = "start_frame_mismatch";
    return false;
  }

  const auto& start_point = request_start.pose.position;
  if (!std::isfinite(start_point.x) || !std::isfinite(start_point.y) ||
      !std::isfinite(start_point.z)) {
    error = "non_finite_start_pose";
    return false;
  }
  for (const auto& pose : input.poses) {
    const auto& point = pose.pose.position;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      error = "non_finite_global_path";
      return false;
    }
  }

  std::size_t closest_index = 0;
  double closest_distance = distance(input.poses.front().pose.position,
                                     start_point);
  for (std::size_t index = 1; index < input.poses.size(); ++index) {
    const double candidate_distance =
        distance(input.poses[index].pose.position, start_point);
    if (candidate_distance < closest_distance) {
      closest_index = index;
      closest_distance = candidate_distance;
    }
  }

  if (closest_distance > max_start_distance_) {
    error = "global_path_too_far_from_robot";
    return false;
  }

  output.header = input.header;
  output.header.frame_id = navigation_frame_;
  output.poses.reserve(input.poses.size() - closest_index);

  geometry_msgs::PoseStamped current = request_start;
  current.header = output.header;
  output.poses.push_back(current);
  for (std::size_t index = closest_index + 1; index < input.poses.size();
       ++index) {
    geometry_msgs::PoseStamped pose = input.poses[index];
    pose.header = output.header;
    output.poses.push_back(pose);
  }

  if (output.poses.size() < 2) {
    output = nav_msgs::Path();
    error = "global_path_already_complete";
    return false;
  }
  return true;
}

}  // namespace navigation_supervisor
