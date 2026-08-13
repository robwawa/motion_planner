#!/usr/bin/env python3
"""Coordinates PCT global planning with SCAN reference-path tracking."""

import math
import threading

import actionlib
import rospy
import tf2_geometry_msgs  # registers PoseStamped conversions
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from pct_planner.msg import PlanPath3DAction, PlanPath3DGoal
from std_msgs.msg import String


class NavigationManager:
    def __init__(self):
        self.frame = rospy.get_param('~navigation_frame', 'map')
        self.body_pose_topic = rospy.get_param('~body_pose_topic', '/quad_0/body_pose')
        self.goal_topic = rospy.get_param('~goal_topic', '/goal_pose_3d')
        # RViz's standard SetGoal tool publishes a 2D PoseStamped here.  Keep
        # it as a compatibility input and lift it into the 3D goal interface.
        self.rviz_goal_topic = rospy.get_param('~rviz_goal_topic', '/move_base_simple/goal')
        self.default_goal_z = rospy.get_param('~default_goal_z', 0.4)
        self.reference_topic = rospy.get_param('~reference_path_topic', '/navigation/reference_path')
        self.validated_goal_topic = rospy.get_param('~validated_goal_topic', '/navigation/validated_goal')
        self.max_start_distance = rospy.get_param('~max_start_distance', 1.0)
        self.action_wait_timeout = rospy.get_param('~action_wait_timeout', 15.0)
        self.odom = None
        self.lock = threading.Lock()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.reference_pub = rospy.Publisher(self.reference_topic, Path, latch=True, queue_size=1)
        self.validated_goal_pub = rospy.Publisher(self.validated_goal_topic, PoseStamped,
                                                  latch=True, queue_size=1)
        self.status_pub = rospy.Publisher(rospy.get_param('~status_topic', '/navigation/status'), String,
                                          latch=True, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.body_pose_topic, Odometry, self.odom_callback, queue_size=10)
        self.goal_sub = rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=1)
        self.rviz_goal_sub = rospy.Subscriber(self.rviz_goal_topic, PoseStamped,
                                              self.rviz_goal_callback, queue_size=1)
        self.client = actionlib.SimpleActionClient('/pct/plan_path', PlanPath3DAction)
        self.publish_status('IDLE')

    def publish_status(self, state, detail=''):
        self.status_pub.publish(String(data=state + (':' + detail if detail else '')))

    def odom_callback(self, msg):
        with self.lock:
            self.odom = msg

    def transform_pose(self, pose):
        if not pose.header.frame_id:
            raise ValueError('pose frame_id is empty')
        if pose.header.frame_id == self.frame:
            return pose
        return self.tf_buffer.transform(pose, self.frame, rospy.Duration(0.5))

    def rviz_goal_callback(self, msg):
        """Convert RViz's 2D SetGoal message into a 3D navigation goal.

        RViz does not provide a Z coordinate with SetGoal.  Preserve an
        explicitly supplied non-zero Z; otherwise keep the robot on its
        current floor by using the latest body height.
        """
        with self.lock:
            odom = self.odom
        goal = PoseStamped()
        goal.header = msg.header
        goal.pose = msg.pose
        if not math.isfinite(goal.pose.position.z) or abs(goal.pose.position.z) < 1e-6:
            goal.pose.position.z = (odom.pose.pose.position.z
                                    if odom is not None else self.default_goal_z)
        self.goal_callback(goal)

    def goal_callback(self, msg):
        with self.lock:
            odom = self.odom
        if odom is None:
            self.publish_status('REJECTED', 'no_odometry')
            return
        try:
            goal = self.transform_pose(msg)
            start = PoseStamped()
            start.header = odom.header
            start.pose = odom.pose.pose
            start = self.transform_pose(start)
        except Exception as exc:
            self.publish_status('REJECTED', 'tf:' + str(exc))
            return
        if not self.client.wait_for_server(rospy.Duration(self.action_wait_timeout)):
            self.publish_status('ABORTED', 'pct_action_unavailable')
            return
        request = PlanPath3DGoal(start=start, goal=goal)
        self.publish_status('GLOBAL_PLANNING')
        self.client.send_goal(request, done_cb=self.plan_done)

    def plan_done(self, state, result):
        if result is None:
            self.publish_status('ABORTED', 'no_action_result')
            return

        # Endpoint projection is useful even when the global route is
        # unavailable: reflect the snapped target in RViz, but never send an
        # empty/failed route to SCAN.
        if result.has_snapped_goal:
            self.validated_goal_pub.publish(result.snapped_goal)

        if result.status != result.SUCCESS:
            reason = result.message if result is not None else 'no_action_result'
            self.publish_status('ABORTED', reason)
            return
        try:
            path = self.prepare_path(result.path)
            self.reference_pub.publish(path)
            self.publish_status('PATH_TRACKING')
        except ValueError as exc:
            self.publish_status('ABORTED', str(exc))

    def prepare_path(self, path):
        if path.header.frame_id != self.frame or len(path.poses) < 2:
            raise ValueError('invalid_global_path')
        points = []
        for pose in path.poses:
            p = pose.pose.position
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                raise ValueError('non_finite_global_path')
            points.append(pose)
        with self.lock:
            odom = self.odom
        if odom is None:
            raise ValueError('odometry_lost')
        current = PoseStamped()
        current.header.frame_id = self.frame
        current.header.stamp = rospy.Time.now()
        current.pose = odom.pose.pose
        closest = min(range(len(points)), key=lambda i: self.distance(points[i].pose.position, current.pose.position))
        if self.distance(points[closest].pose.position, current.pose.position) > self.max_start_distance:
            raise ValueError('global_path_too_far_from_robot')
        output = Path()
        output.header.frame_id = self.frame
        output.header.stamp = rospy.Time.now()
        output.poses = [current] + points[closest + 1:]
        if len(output.poses) < 2:
            raise ValueError('global_path_already_complete')
        for pose in output.poses:
            pose.header = output.header
        return output

    @staticmethod
    def distance(a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


if __name__ == '__main__':
    rospy.init_node('navigation_manager')
    NavigationManager()
    rospy.spin()
