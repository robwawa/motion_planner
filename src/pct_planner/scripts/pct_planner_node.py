#!/usr/bin/env python3
"""Online ROS Action wrapper for the PCT global planner."""

import argparse
import ctypes
import math
import os
import sys

import actionlib
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

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
    TOMOGRAMS = {'Spiral': 'spiral0.3_2', 'Building': 'building2_9', 'Plaza': 'plaza3_10'}

    def __init__(self, scene, navigation_frame, body_height, layer_tolerance, optimize_path,
                 endpoint_snap_radius):
        if scene not in self.TOMOGRAMS:
            raise ValueError('Unknown scene: {}'.format(scene))
        package_root = package_path()
        add_planner_paths(package_root)
        from config import Config
        from planner_wrapper import TomogramPlanner

        self.navigation_frame = navigation_frame
        self.body_height = body_height
        self.layer_tolerance = layer_tolerance
        self.optimize_path = optimize_path
        self.path_pub = rospy.Publisher('/pct/global_path', Path, latch=True, queue_size=1)
        self.planner = TomogramPlanner(Config())
        self.planner.loadTomogram(self.TOMOGRAMS[scene])
        self.endpoint_snap_radius_cells = max(
            0, int(math.ceil(float(endpoint_snap_radius) / self.planner.resolution)))
        self.server = actionlib.SimpleActionServer(
            '/pct/plan_path', PlanPath3DAction, execute_cb=self.execute, auto_start=False)
        self.server.start()
        rospy.loginfo('[pct_planner] ready: scene=%s frame=%s', scene, navigation_frame)

    def feedback(self, stage):
        self.server.publish_feedback(PlanPath3DFeedback(stage=stage))

    @staticmethod
    def finite_pose(pose):
        return all(math.isfinite(value) for value in (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

    def fail(self, status, message):
        result = PlanPath3DResult(status=status, message=message)
        self.server.set_aborted(result, message)

    def execute(self, goal):
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
            start_pose, start_layer, start_distance = start
            goal_pose, goal_layer, goal_distance = end
            self.feedback('planning')
            trajectory = self.planner.plan(
                start_pose[:2], goal_pose[:2], start_layer, goal_layer, self.body_height,
                optimize_path=self.optimize_path)
            if self.server.is_preempt_requested():
                self.server.set_preempted(PlanPath3DResult(status=PlanPath3DResult.PREEMPTED), 'preempted')
                return
            if trajectory is None or len(trajectory) < 2:
                self.fail(PlanPath3DResult.NO_PATH, 'PCT did not find a path')
                return
            self.feedback('publishing')
            path = self.to_path(trajectory, goal.goal)
            # Preserve the exact snapped endpoint in the public path.  This
            # avoids exposing a numerically close, but visually floating,
            # endpoint when the native optimiser returns a rounded value.
            path.poses[-1].pose.position.x = goal_pose[0]
            path.poses[-1].pose.position.y = goal_pose[1]
            path.poses[-1].pose.position.z = goal_pose[2]
            self.path_pub.publish(path)
            result = PlanPath3DResult(
                status=PlanPath3DResult.SUCCESS,
                message='ok; start snapped {:.2f}m, goal snapped {:.2f}m'.format(
                    start_distance, goal_distance), path=path)
            self.server.set_succeeded(result, 'path found')
        except ValueError as exc:
            self.fail(PlanPath3DResult.OUT_OF_MAP, str(exc))
        except Exception as exc:  # native extension errors must not crash a navigation client
            rospy.logerr('[pct_planner] planning failed: %s', exc)
            self.fail(PlanPath3DResult.INTERNAL_ERROR, str(exc))

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
    parser.add_argument('--scene', default='Building')
    args, _ = parser.parse_known_args()
    rospy.init_node('pct_planner')
    scene = rospy.get_param('~scene', args.scene)
    PCTActionServer(scene, rospy.get_param('~navigation_frame', 'map'),
                    rospy.get_param('~body_height', 0.4),
                    rospy.get_param('~layer_height_tolerance', 0.75),
                    rospy.get_param('~optimize_path', True),
                    rospy.get_param('~endpoint_snap_radius', 1.5))
    rospy.spin()


if __name__ == '__main__':
    main()
