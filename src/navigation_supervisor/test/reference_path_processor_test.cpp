#include <gtest/gtest.h>

#include <initializer_list>
#include <limits>

#include "navigation_supervisor/reference_path_processor.h"

namespace {

geometry_msgs::PoseStamped pose(double x, double y, double z = 0.0) {
  geometry_msgs::PoseStamped output;
  output.header.frame_id = "map";
  output.pose.position.x = x;
  output.pose.position.y = y;
  output.pose.position.z = z;
  output.pose.orientation.w = 1.0;
  return output;
}

nav_msgs::Path path(std::initializer_list<double> xs) {
  nav_msgs::Path output;
  output.header.frame_id = "map";
  output.header.stamp = ros::Time(42.0);
  for (const double x : xs) output.poses.push_back(pose(x, 0.0));
  return output;
}

}  // namespace

TEST(ReferencePathProcessor, InsertsCurrentStartAndKeepsRemainingPath) {
  navigation_supervisor::ReferencePathProcessor processor("map", 1.0);
  nav_msgs::Path output;
  std::string error;
  ASSERT_TRUE(processor.process(path({0.0, 1.0, 2.0}), pose(0.8, 0.0),
                               output, error));
  ASSERT_EQ(2u, output.poses.size());
  EXPECT_EQ("map", output.header.frame_id);
  EXPECT_EQ(ros::Time(42.0), output.header.stamp);
  EXPECT_EQ("map", output.poses.front().header.frame_id);
  EXPECT_DOUBLE_EQ(0.8, output.poses.front().pose.position.x);
  EXPECT_DOUBLE_EQ(2.0, output.poses.back().pose.position.x);
}

TEST(ReferencePathProcessor, RejectsPathWithTooFewPoints) {
  navigation_supervisor::ReferencePathProcessor processor("map", 1.0);
  nav_msgs::Path output;
  std::string error;
  EXPECT_FALSE(processor.process(path({0.0}), pose(0.0, 0.0), output, error));
  EXPECT_EQ("invalid_global_path", error);
}

TEST(ReferencePathProcessor, RejectsPathTooFarFromStart) {
  navigation_supervisor::ReferencePathProcessor processor("map", 0.5);
  nav_msgs::Path output;
  std::string error;
  EXPECT_FALSE(processor.process(path({0.0, 1.0}), pose(3.0, 0.0),
                                 output, error));
  EXPECT_EQ("global_path_too_far_from_robot", error);
}

TEST(ReferencePathProcessor, RejectsFrameMismatch) {
  navigation_supervisor::ReferencePathProcessor processor("map", 1.0);
  nav_msgs::Path input = path({0.0, 1.0});
  input.header.frame_id = "odom";
  nav_msgs::Path output;
  std::string error;
  EXPECT_FALSE(processor.process(input, pose(0.0, 0.0), output, error));
  EXPECT_EQ("invalid_global_path", error);
}

TEST(ReferencePathProcessor, RejectsStartFrameMismatch) {
  navigation_supervisor::ReferencePathProcessor processor("map", 1.0);
  nav_msgs::Path output;
  std::string error;
  geometry_msgs::PoseStamped start = pose(0.0, 0.0);
  start.header.frame_id = "odom";
  EXPECT_FALSE(processor.process(path({0.0, 1.0}), start, output, error));
  EXPECT_EQ("start_frame_mismatch", error);
}

TEST(ReferencePathProcessor, RejectsNonFinitePath) {
  navigation_supervisor::ReferencePathProcessor processor("map", 1.0);
  nav_msgs::Path input = path({0.0, 1.0});
  input.poses.back().pose.position.x = std::numeric_limits<double>::quiet_NaN();
  nav_msgs::Path output;
  std::string error;
  EXPECT_FALSE(processor.process(input, pose(0.0, 0.0), output, error));
  EXPECT_EQ("non_finite_global_path", error);
}

TEST(ReferencePathProcessor, RejectsNonFiniteStart) {
  navigation_supervisor::ReferencePathProcessor processor("map", 1.0);
  nav_msgs::Path output;
  std::string error;
  geometry_msgs::PoseStamped start = pose(0.0, 0.0);
  start.pose.position.y = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(processor.process(path({0.0, 1.0}), start, output, error));
  EXPECT_EQ("non_finite_start_pose", error);
}

TEST(ReferencePathProcessor, ReportsAlreadyCompletePath) {
  navigation_supervisor::ReferencePathProcessor processor("map", 1.0);
  nav_msgs::Path output;
  std::string error;
  EXPECT_FALSE(processor.process(path({0.0, 1.0}), pose(1.0, 0.0), output, error));
  EXPECT_EQ("global_path_already_complete", error);
  EXPECT_TRUE(output.poses.empty());
}
