#include <cmath>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>

#include <global_pct_planner/PctTerrainMap.h>
#include <motion_planner_log/logging.h>
#include "global_pct_planner/tomogram.h"
#include "global_pct_planner/tomogram_io.h"

namespace global_pct_planner {
namespace {

struct CloudPoint {
  PointXYZ point;
  float intensity{0.0f};
};

TomogramProfile readProfile() {
  TomogramProfile profile;
  ros::param::param("pct/map/resolution", profile.resolution, 0.1);
  ros::param::param("pct/map/ground_h", profile.ground_h, 0.0);
  ros::param::param("pct/map/slice_dh", profile.slice_dh, 0.5);
  ros::param::param("pct/traversability/kernel_size", profile.kernel_size, 7);
  ros::param::param("pct/traversability/interval_min", profile.interval_min,
                   0.5);
  ros::param::param("pct/traversability/interval_free", profile.interval_free,
                   0.5);
  ros::param::param("pct/traversability/slope_max", profile.slope_max, 0.4);
  ros::param::param("pct/traversability/step_max", profile.step_max, 0.3);
  ros::param::param("pct/traversability/standable_ratio",
                   profile.standable_ratio, 0.5);
  ros::param::param("pct/traversability/cost_barrier", profile.cost_barrier,
                   50.0);
  ros::param::param("pct/traversability/safe_margin", profile.safe_margin,
                   0.5);
  ros::param::param("pct/traversability/inflation", profile.inflation, 0.1);
  ros::param::param("pct/traversability/cost_threshold",
                   profile.cost_threshold, 49.0);
  return profile;
}

sensor_msgs::PointCloud2 makeCloud(const std::vector<CloudPoint>& points,
                                   const std::string& frame,
                                   const ros::Time& stamp) {
  sensor_msgs::PointCloud2 message;
  message.header.frame_id = frame;
  message.header.stamp = stamp;
  message.height = 1;
  message.width = static_cast<uint32_t>(points.size());
  message.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
      sensor_msgs::PointField::FLOAT32, "z", 1,
      sensor_msgs::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::PointField::FLOAT32);
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(message, "x");
  sensor_msgs::PointCloud2Iterator<float> y(message, "y");
  sensor_msgs::PointCloud2Iterator<float> z(message, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(message, "intensity");
  for (const CloudPoint& point : points) {
    *x = point.point.x;
    *y = point.point.y;
    *z = point.point.z;
    *intensity = point.intensity;
    ++x;
    ++y;
    ++z;
    ++intensity;
  }
  return message;
}

void publishMap(const TomogramData& map, const std::string& frame,
                const std::string& ground_topic, const std::string& ceiling_topic,
                ros::Publisher& terrain_publisher,
                ros::Publisher& tomogram_publisher,
                std::vector<ros::Publisher>& ground_publishers,
                std::vector<ros::Publisher>& ceiling_publishers) {
  const ros::Time stamp = ros::Time::now();
  PctTerrainMap terrain;
  terrain.header.frame_id = frame;
  terrain.header.stamp = stamp;
  terrain.resolution = map.resolution;
  terrain.center_x = map.center_x;
  terrain.center_y = map.center_y;
  terrain.rows = map.rows;
  terrain.cols = map.cols;
  terrain.layers = map.layers;
  terrain.traversability = map.traversability;
  // The legacy ROS message publishes nan_to_num(layers_g, nan=0.0), while
  // elevation_valid preserves the original finite mask.  Keep .pctm raw and
  // apply this conversion only to the wire-format message.
  terrain.ground_elevation.resize(map.cellCount());
  terrain.elevation_valid.resize(map.cellCount(), 0);
  for (std::size_t i = 0; i < map.cellCount(); ++i) {
    if (std::isnan(map.ground_elevation[i])) {
      terrain.ground_elevation[i] = 0.0f;
    } else if (std::isinf(map.ground_elevation[i])) {
      terrain.ground_elevation[i] =
          map.ground_elevation[i] > 0.0f
              ? std::numeric_limits<float>::max()
              : -std::numeric_limits<float>::max();
    } else {
      terrain.ground_elevation[i] = map.ground_elevation[i];
    }
    terrain.elevation_valid[i] = std::isfinite(map.ground_elevation[i]) ? 1 : 0;
  }
  terrain_publisher.publish(terrain);

  std::vector<CloudPoint> all_ground;
  all_ground.reserve(map.cellCount());
  ground_publishers.clear();
  ceiling_publishers.clear();
  ros::NodeHandle nh;
  for (uint32_t layer = 0; layer < map.layers; ++layer) {
    std::vector<CloudPoint> ground;
    std::vector<CloudPoint> ceiling;
    ground.reserve(static_cast<std::size_t>(map.rows) * map.cols / 4);
    ceiling.reserve(static_cast<std::size_t>(map.rows) * map.cols / 4);
    for (uint32_t row = 0; row < map.rows; ++row) {
      for (uint32_t col = 0; col < map.cols; ++col) {
        const std::size_t index = map.index(layer, row, col);
        const float x = (static_cast<int>(row) -
                         static_cast<int>(map.rows / 2)) * map.resolution +
                        map.center_x;
        const float y = (static_cast<int>(col) -
                         static_cast<int>(map.cols / 2)) * map.resolution +
                        map.center_y;
        if (std::isfinite(map.ground_elevation[index])) {
          const CloudPoint point{{x, y, map.ground_elevation[index]},
                                 map.traversability[index]};
          ground.push_back(point);
          all_ground.push_back(point);
        }
        if (std::isfinite(map.ceiling_elevation[index])) {
          ceiling.push_back({{x, y, map.ceiling_elevation[index]},
                             map.traversability[index]});
        }
      }
    }
    ground_publishers.push_back(nh.advertise<sensor_msgs::PointCloud2>(
        ground_topic + std::to_string(layer), 1, true));
    ceiling_publishers.push_back(nh.advertise<sensor_msgs::PointCloud2>(
        ceiling_topic + std::to_string(layer), 1, true));
    ground_publishers.back().publish(makeCloud(ground, frame, stamp));
    ceiling_publishers.back().publish(makeCloud(ceiling, frame, stamp));
  }
  tomogram_publisher.publish(makeCloud(all_ground, frame, stamp));
}

}  // namespace
}  // namespace global_pct_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "pct_tomography_node");
  motion_planner_log::initialize("global_pct_tomography", argv[0]);
  ros::NodeHandle private_handle("~");

  std::string pcd_file;
  std::string tomogram_name;
  std::string export_dir;
  std::string frame;
  private_handle.param<std::string>("pcd_file", pcd_file, "");
  private_handle.param<std::string>("tomogram_name", tomogram_name,
                                    "building2_9");
  private_handle.param<std::string>("export_dir", export_dir, "");
  ros::param::param("pointcloud_tomography/map_frame", frame,
                   std::string("map"));
  if (export_dir.empty()) {
    MOTION_PLANNER_LOG_FATAL(
        "~export_dir is empty; set the tomogram write directory in the launch "
        "file, for example src/global_pct_planner/rsc/tomogram");
    return 2;
  }
  if (!export_dir.empty() && export_dir.back() != '/') export_dir += '/';
  const std::string output_path = export_dir + tomogram_name + ".pctm";

  global_pct_planner::TomogramData map;
  std::string error;
  bool loaded = false;
  if (pcd_file.empty() && global_pct_planner::TomogramIO::load(
                              output_path, map, &error)) {
    loaded = true;
  }

  std::vector<global_pct_planner::PointXYZ> points;
  if (!loaded) {
    if (pcd_file.empty()) {
      MOTION_PLANNER_LOG_FATAL(
          "No cached .pctm map at %s and ~pcd_file is empty; set the PCD path "
          "in the launch file", output_path.c_str());
      return 2;
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, cloud) != 0) {
      MOTION_PLANNER_LOG_FATAL("Unable to load PCD file: %s", pcd_file.c_str());
      return 3;
    }
    points.reserve(cloud.size());
    for (const pcl::PointXYZ& point : cloud.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) &&
          std::isfinite(point.z)) {
        points.push_back({point.x, point.y, point.z});
      }
    }
    global_pct_planner::TomogramEngine engine(
        global_pct_planner::readProfile());
    global_pct_planner::TomogramTimings timings;
    map = engine.generate(points, &timings);
    if (!global_pct_planner::TomogramIO::save(output_path, map, &error)) {
      MOTION_PLANNER_LOG_ERROR("Unable to save .pctm map at %s: %s",
                               output_path.c_str(), error.c_str());
    } else {
      MOTION_PLANNER_LOG_INFO("Saved .pctm map: %s", output_path.c_str());
    }
    MOTION_PLANNER_LOG_INFO(
        "Tomography generated: points=%zu dimensions=%ux%ux%u total=%.2f ms",
        points.size(), map.layers, map.rows, map.cols, timings.total_ms);
  } else {
    MOTION_PLANNER_LOG_INFO("Loaded cached .pctm map: %s", output_path.c_str());
  }

  std::string terrain_topic;
  std::string tomogram_topic;
  std::string ground_topic;
  std::string ceiling_topic;
  ros::param::param("pointcloud_tomography/terrain_map_topic", terrain_topic,
                   std::string("/pct/terrain_map"));
  ros::param::param("pointcloud_tomography/tomogram_topic", tomogram_topic,
                   std::string("/tomogram"));
  ros::param::param("pointcloud_tomography/layer_G_topic", ground_topic,
                   std::string("/layer_G_"));
  ros::param::param("pointcloud_tomography/layer_C_topic", ceiling_topic,
                   std::string("/layer_C_"));
  ros::NodeHandle nh;
  ros::Publisher terrain_publisher =
      nh.advertise<global_pct_planner::PctTerrainMap>(terrain_topic, 1, true);
  ros::Publisher tomogram_publisher =
      nh.advertise<sensor_msgs::PointCloud2>(tomogram_topic, 1, true);
  std::vector<ros::Publisher> ground_publishers;
  std::vector<ros::Publisher> ceiling_publishers;
  global_pct_planner::publishMap(
      map, frame, ground_topic, ceiling_topic,
      terrain_publisher, tomogram_publisher, ground_publishers,
      ceiling_publishers);

  std::string pointcloud_topic;
  ros::param::param("pointcloud_tomography/pointcloud_topic", pointcloud_topic,
                   std::string("/global_points"));
  ros::Publisher raw_points_publisher =
      nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 1, true);
  if (!points.empty()) {
    std::vector<global_pct_planner::CloudPoint> raw;
    raw.reserve(points.size());
    for (const auto& point : points) raw.push_back({point, 1.0f});
    raw_points_publisher.publish(
        global_pct_planner::makeCloud(raw, frame, ros::Time::now()));
  }
  MOTION_PLANNER_LOG_INFO(
      "Ready: map=%s frame=%s terrain_topic=%s tomogram_topic=%s",
      output_path.c_str(), frame.c_str(), terrain_topic.c_str(),
      tomogram_topic.c_str());
  ros::spin();
  return 0;
}
