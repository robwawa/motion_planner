#!/usr/bin/env python3
"""Online ROS Action wrapper for the PCT global planner."""

import argparse
import ctypes
import math
import os
import sys
import time

import actionlib
import numpy as np
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool, Header, UInt64

from pct_planner.msg import PlanPath3DAction, PlanPath3DFeedback, PlanPath3DResult


def package_path():
    try:
        return rospkg.RosPack().get_path('pct_planner')
    except rospkg.common.ResourceNotFound:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def add_planner_paths(package_root):
    planner_root = os.path.join(package_root, 'planner')
    script_dir = os.path.join(planner_root, 'scripts')
    lib_dir = os.path.join(planner_root, 'lib')
    native_dirs = [
        os.path.join(lib_dir, 'build', 'src', 'a_star'),
        os.path.join(lib_dir, 'build', 'src', 'map_manager'),
        os.path.join(lib_dir, 'build', 'src', 'trajectory_optimization'),
        os.path.join(lib_dir, 'build', 'src', 'ele_planner'),
        os.path.join(lib_dir, 'build', 'src', 'common', 'smoothing'),
        os.path.join(lib_dir, '3rdparty', 'gtsam-4.1.1', 'install', 'lib'),
        os.path.join(lib_dir, '3rdparty', 'gtsam-4.1.1', 'build', 'gtsam', '3rdparty', 'metis', 'libmetis'),
    ]
    old_ld = os.environ.get('LD_LIBRARY_PATH', '')
    os.environ['LD_LIBRARY_PATH'] = ':'.join(native_dirs + ([old_ld] if old_ld else []))
    # LD_LIBRARY_PATH is read when Python starts, so setting it above cannot
    # repair a dependency of a later extension import.  Preload GTSAM's METIS
    # dependency with global visibility for both rosrun and roslaunch usage.
    metis = os.path.join(lib_dir, '3rdparty', 'gtsam-4.1.1', 'install', 'lib', 'libmetis-gtsam.so')
    if os.path.isfile(metis):
        ctypes.CDLL(metis, mode=ctypes.RTLD_GLOBAL)
    sys.path.insert(0, script_dir)
    sys.path.insert(0, planner_root)


class PCTActionServer:
    def __init__(self, navigation_frame, body_height, layer_tolerance, optimize_path,
                 endpoint_snap_radius, tomogram_name, wait_timeout=300.0):
        if not tomogram_name:
            raise ValueError('tomogram_name must be provided')
        package_root = package_path()
        add_planner_paths(package_root)
        sys.path.insert(0, os.path.join(package_root, 'tomography', 'config'))
        from config import Config
        from planner_wrapper import TomogramPlanner
        from pct_profile import load_public_profile

        self.navigation_frame = navigation_frame
        self.body_height = body_height
        self.layer_tolerance = layer_tolerance
        self.optimize_path = optimize_path
        self.path_pub = rospy.Publisher('/pct/global_path', Path, latch=True, queue_size=1)
        self.planner = TomogramPlanner(Config())
        self.planner.traversable_cost_threshold = load_public_profile().trav.cost_threshold
        rospy.loginfo('[pct_planner] public traversability cost_threshold=%.3f',
                      self.planner.traversable_cost_threshold)
        self.tomogram_name = tomogram_name
        tomogram_path = os.path.join(package_root, 'rsc', 'tomogram',
                                     self.tomogram_name + '.pickle')
        deadline = time.monotonic() + max(0.0, float(wait_timeout))
        while not os.path.isfile(tomogram_path):
            if time.monotonic() >= deadline:
                raise RuntimeError('timed out waiting for tomogram: {}'.format(tomogram_path))
            rospy.loginfo_throttle(5.0, '[pct_planner] waiting for tomogram: %s', tomogram_path)
            time.sleep(0.2)
        self.planner.loadTomogram(self.tomogram_name)
        self.dynamic_enabled = rospy.get_param('~dynamic_replan/enabled', False)
        self.dynamic_layer = None
        self.last_dynamic_snapshot_received = 0.0
        self.dynamic_source_timeout = 0.5
        self.dynamic_source_healthy = False
        self.max_debug_points = int(rospy.get_param(
            '~dynamic_replan/max_debug_points', 100000))
        self.dynamic_ok_pub = rospy.Publisher(
            '/pct/dynamic_layer_ok', Bool, latch=True, queue_size=1)
        self.dynamic_version_pub = rospy.Publisher(
            '/pct/dynamic_version', UInt64, latch=True, queue_size=1)
        self.dynamic_cloud_pub = rospy.Publisher(
            '/pct/dynamic_cloud', PointCloud2, queue_size=1)
        self.dynamic_cost_pub = rospy.Publisher(
            '/pct/dynamic_costmap', PointCloud2, queue_size=1)
        if self.dynamic_enabled:
            try:
                self._init_dynamic_replan()
            except (MemoryError, ValueError) as exc:
                rospy.logerr('[pct_planner] dynamic replan disabled: %s', exc)
                self.dynamic_enabled = False
                self.dynamic_ok_pub.publish(Bool(data=False))
        else:
            self.dynamic_ok_pub.publish(Bool(data=True))
        self.endpoint_snap_radius_cells = max(
            0, int(math.ceil(float(endpoint_snap_radius) / self.planner.resolution)))
        self.server = actionlib.SimpleActionServer(
            '/pct/plan_path', PlanPath3DAction, execute_cb=self.execute, auto_start=False)
        self.server.start()
        rospy.loginfo('[pct_planner] ready: tomogram=%s frame=%s',
                      self.tomogram_name, navigation_frame)

    def _init_dynamic_replan(self):
        from dynamic_obstacle_layer import DynamicObstacleLayer

        p = '~dynamic_replan/'
        lethal = int(rospy.get_param(p + 'lethal_cost', 100))
        collision_top = self.body_height + float(rospy.get_param(
            p + 'collision_top_margin', 0.20))
        self.dynamic_layer = DynamicObstacleLayer(
            self.planner.trav, self.planner.elev_g, self.planner.elev_c,
            self.planner.center, self.planner.resolution,
            self.planner.traversable_cost_threshold,
            lethal_cost=lethal,
            terrain_ignore_height=rospy.get_param(p + 'terrain_ignore_height', 0.08),
            collision_top=collision_top,
            layer_height_tolerance=rospy.get_param(p + 'layer_height_tolerance', 0.25),
            robot_radius=rospy.get_param(p + 'robot_radius', 0.32),
            safety_margin=rospy.get_param(p + 'safety_margin', 0.15),
            inflation_radius=rospy.get_param(p + 'inflation_radius', 0.50),
            max_points_per_snapshot=rospy.get_param(
                p + 'max_points_per_snapshot', 200000),
            assignment_chunk_size=rospy.get_param(p + 'assignment_chunk_size', 20000),
            max_dynamic_memory_mb=rospy.get_param(p + 'max_dynamic_memory_mb', 128.0))
        self.max_snapshot_retries = max(0, int(rospy.get_param(
            p + 'max_snapshot_retries', 1)))
        self.dynamic_source_timeout = max(0.0, float(rospy.get_param(
            p + 'dynamic_source_timeout', 0.5)))
        cloud_topic = rospy.get_param(p + 'cloud_topic', '/dynamic_perception/dynamic_cloud')
        health_topic = rospy.get_param(p + 'health_topic',
                                       '/dynamic_perception/scan_healthy')
        self.health_sub = rospy.Subscriber(health_topic, Bool, self._dynamic_health_cb,
                                           queue_size=1)
        self.cloud_sub = rospy.Subscriber(cloud_topic, PointCloud2, self._dynamic_cloud_cb,
                                          queue_size=1, buff_size=16 * 1024 * 1024)
        update_rate = max(1.0, float(rospy.get_param(p + 'update_rate', 10.0)))
        debug_rate = max(0.1, float(rospy.get_param(p + 'debug_publish_rate', 1.0)))
        self.decay_timer = rospy.Timer(rospy.Duration(1.0 / update_rate), self._decay_cb)
        self.debug_timer = rospy.Timer(rospy.Duration(1.0 / debug_rate), self._debug_cb)
        self.dynamic_ok_pub.publish(Bool(data=False))
        cells = int(np.prod(self.dynamic_layer.shape, dtype=np.int64))
        rospy.loginfo(
            '[pct_planner] dynamic layer enabled: shape=%s cells=%d state=%.2f MiB',
            self.dynamic_layer.shape, cells, cells * 6.0 / (1024.0 * 1024.0))

    def _dynamic_health_cb(self, msg):
        self.dynamic_source_healthy = bool(msg.data)
        if not self.dynamic_source_healthy:
            self.last_dynamic_snapshot_received = 0.0
            if self.dynamic_layer.clear():
                self.dynamic_version_pub.publish(UInt64(data=self.dynamic_layer.version))
            self.dynamic_ok_pub.publish(Bool(data=False))

    @staticmethod
    def _cloud_xyz_view(msg, max_points=None):
        formats = {
            PointField.INT8: 'i1', PointField.UINT8: 'u1',
            PointField.INT16: 'i2', PointField.UINT16: 'u2',
            PointField.INT32: 'i4', PointField.UINT32: 'u4',
            PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}
        fields = {field.name: field for field in msg.fields}
        if any(name not in fields for name in ('x', 'y', 'z')):
            raise ValueError('PointCloud2 has no x/y/z fields')
        endian = '>' if msg.is_bigendian else '<'
        dtype = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': [endian + formats[fields[name].datatype] for name in ('x', 'y', 'z')],
            'offsets': [fields[name].offset for name in ('x', 'y', 'z')],
            'itemsize': msg.point_step})
        view = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
        if max_points and len(view) > max_points:
            view = view[::int(math.ceil(len(view) / float(max_points)))]
        return np.column_stack((view['x'], view['y'], view['z'])).astype(np.float32, copy=False)

    def _dynamic_cloud_cb(self, msg):
        if not self.dynamic_source_healthy:
            rospy.logwarn_throttle(
                2.0, '[pct_planner] dynamic cloud ignored: perception source is unhealthy')
            return
        try:
            if msg.header.frame_id != self.navigation_frame:
                raise ValueError('dynamic cloud frame {} does not match {}'.format(
                    msg.header.frame_id or '<empty>', self.navigation_frame))
            points = self._cloud_xyz_view(msg, self.dynamic_layer.max_points_per_snapshot)
            self.dynamic_layer.replace_snapshot(points)
            self.last_dynamic_snapshot_received = time.monotonic()
            self.dynamic_version_pub.publish(UInt64(data=self.dynamic_layer.version))
            self.dynamic_ok_pub.publish(Bool(data=True))
        except Exception as exc:
            self.dynamic_ok_pub.publish(Bool(data=False))
            rospy.logwarn_throttle(2.0, '[pct_planner] dynamic cloud rejected: %s', exc)

    def _decay_cb(self, _event):
        if (self.last_dynamic_snapshot_received and
                time.monotonic() - self.last_dynamic_snapshot_received >
                self.dynamic_source_timeout):
            self.last_dynamic_snapshot_received = 0.0
            self.dynamic_source_healthy = False
            if self.dynamic_layer.clear():
                self.dynamic_version_pub.publish(UInt64(data=self.dynamic_layer.version))
            self.dynamic_ok_pub.publish(Bool(data=False))
            rospy.logwarn_throttle(
                2.0, '[pct_planner] dynamic perception source timed out; projection cleared')

    def _debug_cb(self, _event):
        header = Header(stamp=rospy.Time.now(), frame_id=self.navigation_frame)
        if self.dynamic_cloud_pub.get_num_connections() > 0:
            assigned = self.dynamic_layer.last_assigned_points
            if len(assigned):
                if len(assigned) > self.max_debug_points:
                    assigned = assigned[::int(math.ceil(len(assigned) / self.max_debug_points))]
                x, y = self.dynamic_layer.grid_to_world(
                    assigned[:, 1].astype(np.int32), assigned[:, 2].astype(np.int32))
                points = np.column_stack((x, y, assigned[:, 3])).astype(np.float32)
            else:
                points = np.empty((0, 3), dtype=np.float32)
            self.dynamic_cloud_pub.publish(pc2.create_cloud_xyz32(header, points))
        if self.dynamic_cost_pub.get_num_connections() > 0:
            layers, rows, cols, costs = self.dynamic_layer.nonzero_cells(self.max_debug_points)
            if len(rows):
                x, y = self.dynamic_layer.grid_to_world(rows, cols)
                z = self.planner.elev_g[layers, rows, cols] + 0.03
                points = np.column_stack((x, y, z)).astype(np.float32)
            else:
                points = np.empty((0, 3), dtype=np.float32)
            self.dynamic_cost_pub.publish(pc2.create_cloud_xyz32(header, points))

    def feedback(self, stage):
        self.server.publish_feedback(PlanPath3DFeedback(stage=stage))

    @staticmethod
    def finite_pose(pose):
        return all(math.isfinite(value) for value in (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

    def make_snapped_pose(self, source, position):
        pose = PoseStamped()
        pose.header.frame_id = self.navigation_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose = source.pose
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        return pose

    def make_result(self, status, message, path=None, snapped_start=None, snapped_goal=None):
        result = PlanPath3DResult(status=status, message=message)
        if path is not None:
            result.path = path
        if snapped_start is not None:
            pose, distance, _ = snapped_start
            result.has_snapped_start = True
            result.snapped_start = pose
            result.snapped_start_distance = float(distance)
        if snapped_goal is not None:
            pose, distance, _ = snapped_goal
            result.has_snapped_goal = True
            result.snapped_goal = pose
            result.snapped_goal_distance = float(distance)
        return result

    def fail(self, status, message, snapped_start=None, snapped_goal=None):
        result = self.make_result(status, message, snapped_start=snapped_start,
                                  snapped_goal=snapped_goal)
        self.server.set_aborted(result, message)

    def execute(self, goal):
        snapped_start = None
        snapped_goal = None
        self.feedback('validating')
        if (goal.start.header.frame_id != self.navigation_frame or
                goal.goal.header.frame_id != self.navigation_frame):
            self.fail(PlanPath3DResult.FRAME_MISMATCH,
                      'start and goal must be expressed in {}'.format(self.navigation_frame))
            return
        if not self.finite_pose(goal.start.pose) or not self.finite_pose(goal.goal.pose):
            self.fail(PlanPath3DResult.INVALID_REQUEST, 'poses contain non-finite values')
            return
        if self.server.is_preempt_requested():
            self.server.set_preempted(PlanPath3DResult(status=PlanPath3DResult.PREEMPTED), 'preempted')
            return

        try:
            planning_version = 0
            if self.dynamic_enabled:
                with self.dynamic_layer.lock:
                    dynamic_cost, planning_version = self.dynamic_layer.snapshot()
                    self.planner.set_dynamic_cost_snapshot(
                        dynamic_cost, planning_version, self.dynamic_layer.lethal_cost)
            self.feedback('snapping_endpoints')
            start = self.planner.snap_to_traversable(
                (goal.start.pose.position.x, goal.start.pose.position.y, goal.start.pose.position.z),
                reference_height=self.body_height,
                radius_cells=self.endpoint_snap_radius_cells)
            end = self.planner.snap_to_traversable(
                (goal.goal.pose.position.x, goal.goal.pose.position.y, goal.goal.pose.position.z),
                reference_height=self.body_height,
                radius_cells=self.endpoint_snap_radius_cells)
            if start is None or end is None:
                self.fail(PlanPath3DResult.NO_TRAVERSABLE_LAYER,
                          'no traversable surface near start or goal')
                return
            start_position, start_layer, start_distance = start
            goal_position, goal_layer, goal_distance = end
            snapped_start = (self.make_snapped_pose(goal.start, start_position),
                             start_distance, start_layer)
            snapped_goal = (self.make_snapped_pose(goal.goal, goal_position),
                            goal_distance, goal_layer)
            start_index = self.planner.pos2idx(start_position[:2]).astype(int)
            goal_index = self.planner.pos2idx(goal_position[:2]).astype(int)
            rospy.loginfo(
                '[pct_planner] raw start=(%.2f, %.2f, %.2f), goal=(%.2f, %.2f, %.2f); '
                'snapped start=(%.2f, %.2f, %.2f) layer=%d grid=(%d,%d) dist=%.2f; '
                'goal=(%.2f, %.2f, %.2f) layer=%d grid=(%d,%d) dist=%.2f',
                goal.start.pose.position.x, goal.start.pose.position.y, goal.start.pose.position.z,
                goal.goal.pose.position.x, goal.goal.pose.position.y, goal.goal.pose.position.z,
                start_position[0], start_position[1], start_position[2], start_layer,
                start_index[0], start_index[1], start_distance,
                goal_position[0], goal_position[1], goal_position[2], goal_layer,
                goal_index[0], goal_index[1], goal_distance)
            self.feedback('planning')
            planning_start = time.monotonic()
            trajectory = None
            attempts = 1 + self.max_snapshot_retries if self.dynamic_enabled else 1
            for attempt in range(attempts):
                trajectory = self.planner.plan(
                    start_position[:2], goal_position[:2], start_layer, goal_layer,
                    self.body_height, optimize_path=self.optimize_path)
                if not self.dynamic_enabled or self.dynamic_layer.version == planning_version:
                    break
                if attempt + 1 < attempts:
                    with self.dynamic_layer.lock:
                        dynamic_cost, planning_version = self.dynamic_layer.snapshot()
                        self.planner.set_dynamic_cost_snapshot(
                            dynamic_cost, planning_version, self.dynamic_layer.lethal_cost)
                    rospy.loginfo('[pct_planner] dynamic layer changed during planning; retry version %d',
                                  planning_version)
            if self.dynamic_enabled and self.dynamic_layer.version != planning_version:
                rospy.logwarn('[pct_planner] dynamic layer remained unstable after %d retries; '
                              'reject stale path', self.max_snapshot_retries)
                trajectory = None
            planning_elapsed_ms = (time.monotonic() - planning_start) * 1000.0
            if self.server.is_preempt_requested():
                self.server.set_preempted(
                    self.make_result(PlanPath3DResult.PREEMPTED, 'preempted',
                                     snapped_start=snapped_start, snapped_goal=snapped_goal),
                    'preempted')
                return
            if trajectory is None or len(trajectory) < 2:
                rospy.logwarn('[pct_planner] A* failed in %.1f ms.', planning_elapsed_ms)
                self.fail(PlanPath3DResult.NO_PATH, 'PCT did not find a path',
                          snapped_start, snapped_goal)
                return
            rospy.loginfo('[pct_planner] A* found %d points in %.1f ms.',
                          len(trajectory), planning_elapsed_ms)
            self.feedback('publishing')
            path = self.to_path(trajectory, goal.goal)
            # Preserve the exact snapped endpoint in the public path.  This
            # avoids exposing a numerically close, but visually floating,
            # endpoint when the native optimiser returns a rounded value.
            path.poses[-1].pose.position.x = goal_position[0]
            path.poses[-1].pose.position.y = goal_position[1]
            path.poses[-1].pose.position.z = goal_position[2]
            self.path_pub.publish(path)
            result = self.make_result(
                PlanPath3DResult.SUCCESS,
                'ok; start snapped {:.2f}m, goal snapped {:.2f}m'.format(
                    start_distance, goal_distance),
                path, snapped_start, snapped_goal)
            self.server.set_succeeded(result, 'path found')
        except ValueError as exc:
            self.fail(PlanPath3DResult.OUT_OF_MAP, str(exc), snapped_start, snapped_goal)
        except Exception as exc:  # native extension errors must not crash a navigation client
            rospy.logerr('[pct_planner] planning failed: %s', exc)
            self.fail(PlanPath3DResult.INTERNAL_ERROR, str(exc), snapped_start, snapped_goal)

    def to_path(self, trajectory, goal):
        path = Path()
        path.header.frame_id = self.navigation_frame
        path.header.stamp = rospy.Time.now()
        for i, point in enumerate(trajectory):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
            if i + 1 < len(trajectory):
                dx, dy = trajectory[i + 1][0] - point[0], trajectory[i + 1][1] - point[1]
                yaw = math.atan2(dy, dx) if dx or dy else 0.0
                pose.pose.orientation.z = math.sin(yaw * 0.5)
                pose.pose.orientation.w = math.cos(yaw * 0.5)
            else:
                pose.pose.orientation = goal.pose.orientation
            path.poses.append(pose)
        return path


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--tomogram-name', required=True)
    args, _ = parser.parse_known_args()
    rospy.init_node('pct_planner')
    PCTActionServer(rospy.get_param('~navigation_frame', 'map'),
                    rospy.get_param('~body_height', 0.4),
                    rospy.get_param('~layer_height_tolerance', 0.75),
                    rospy.get_param('~optimize_path', True),
                    rospy.get_param('~endpoint_snap_radius', 1.5),
                    rospy.get_param('~tomogram_name', args.tomogram_name),
                    rospy.get_param('~wait_timeout', 300.0))
    rospy.spin()


if __name__ == '__main__':
    main()
