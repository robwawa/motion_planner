#pragma once

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace navigation_supervisor {

class ReferencePathProcessor {
 public:
  ReferencePathProcessor(std::string navigation_frame,
                         double max_start_distance);

  bool process(const nav_msgs::Path& input,
               const geometry_msgs::PoseStamped& request_start,
               nav_msgs::Path& output,
               std::string& error) const;

 private:
  static double distance(const geometry_msgs::Point& lhs,
                         const geometry_msgs::Point& rhs);

  std::string navigation_frame_;
  double max_start_distance_{1.0};
};

}  // namespace navigation_supervisor
