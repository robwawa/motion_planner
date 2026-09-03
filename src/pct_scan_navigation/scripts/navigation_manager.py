#!/usr/bin/env python3
"""Coordinates PCT global planning with SCAN reference-path tracking."""

import math
import os
import sys
import threading

import actionlib
import rospy
import tf2_geometry_msgs  # registers PoseStamped conversions
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from pct_planner.msg import PlanPath3DAction, PlanPath3DGoal
from std_msgs.msg import Empty, String

# catkin's devel-space executable is a relay script, so its directory is not
# this source file's directory. Keep local helper modules importable in both
# direct and relay execution.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from replan_cycle_budget import ReplanCycleBudget


class NavigationManager:
    """Own the PCT Action lifecycle; SCAN owns local-replan exhaustion."""

    def __init__(self):
        self.frame = rospy.get_param('~navigation_frame', 'map')
        self.body_pose_topic = rospy.get_param('~body_pose_topic', '/quad_0/body_pose')
        self.goal_topic = rospy.get_param('~goal_topic', '/goal_pose_3d')
        self.rviz_goal_topic = rospy.get_param('~rviz_goal_topic', '/move_base_simple/goal')
        self.default_goal_z = rospy.get_param('~default_goal_z', 0.4)
        self.reference_topic = rospy.get_param('~reference_path_topic', '/navigation/reference_path')
        self.validated_goal_topic = rospy.get_param('~validated_goal_topic', '/navigation/validated_goal')
        self.max_start_distance = rospy.get_param('~max_start_distance', 1.0)
        self.action_wait_timeout = rospy.get_param('~action_wait_timeout', 15.0)
        self.scan_replan_request_topic = rospy.get_param(
            '~scan_replan_request_topic', '/scan/global_replan_request')
        self.goal_reached_distance = float(rospy.get_param('~goal_reached_distance', 0.5))
        self.replan_max_attempts = max(1, int(rospy.get_param('~replan_max_attempts', 3)))
        self.replan_retry_delays = [max(0.0, float(delay)) for delay in rospy.get_param(
            '~replan_retry_delays', [2.0, 4.0])]
        self.global_replan_max_cycles = max(1, int(rospy.get_param(
            '~global_replan_max_cycles', 3)))

        self.odom = None
        self.active_goal = None       # Original user target, used for every PCT request.
        self.terminal_goal = None     # Latest PCT-snapped endpoint, used for arrival.
        self.planning = False
        self.request_generation = 0
        self.replan_attempt = 0
        self.replan_cycle_budget = ReplanCycleBudget(self.global_replan_max_cycles)
        self.retry_timer = None
        self.lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.reference_pub = rospy.Publisher(self.reference_topic, Path, latch=True, queue_size=1)
        self.validated_goal_pub = rospy.Publisher(
            self.validated_goal_topic, PoseStamped, latch=True, queue_size=1)
        self.status_pub = rospy.Publisher(
            rospy.get_param('~status_topic', '/navigation/status'), String, latch=True, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.body_pose_topic, Odometry, self.odom_callback, queue_size=10)
        self.goal_sub = rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=1)
        self.rviz_goal_sub = rospy.Subscriber(self.rviz_goal_topic, PoseStamped,
                                              self.rviz_goal_callback, queue_size=1)
        self.scan_replan_sub = rospy.Subscriber(
            self.scan_replan_request_topic, Empty, self.scan_replan_request_callback, queue_size=1)
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
        with self.lock:
            odom = self.odom
        goal = PoseStamped()
        goal.header = msg.header
        goal.pose = msg.pose
        if not math.isfinite(goal.pose.position.z) or abs(goal.pose.position.z) < 1e-6:
            goal.pose.position.z = (odom.pose.pose.position.z
                                    if odom is not None else self.default_goal_z)
        self.goal_callback(goal)

    def _cancel_retry_locked(self):
        if self.retry_timer is not None:
            self.retry_timer.shutdown()
            self.retry_timer = None

    def goal_callback(self, msg):
        with self.lock:
            odom = self.odom
        if odom is None:
            self.publish_status('REJECTED', 'no_odometry')
            return
        try:
            goal = self.transform_pose(msg)
        except Exception as exc:
            self.publish_status('REJECTED', 'tf:' + str(exc))
            return

        with self.lock:
            self._cancel_retry_locked()
            self.active_goal = goal
            self.terminal_goal = None
            self.replan_attempt = 0
            self.replan_cycle_budget.reset()
            self.request_generation += 1
            generation = self.request_generation
            was_planning = self.planning
            self.planning = False
        if was_planning:
            self.client.cancel_goal()
        self.send_plan(goal, generation, is_replan=False)

    def scan_replan_request_callback(self, _msg):
        """One SCAN event follows exhaustion of its local replan budget."""
        with self.lock:
            if self.active_goal is None or self.odom is None:
                status, goal = ('REPLAN_IGNORED', 'no_active_goal'), None
            elif self.planning or self.retry_timer is not None:
                status, goal = ('REPLAN_IGNORED', 'already_pending'), None
            else:
                terminal = self.terminal_goal or self.active_goal
                current = self.odom.pose.pose.position
                if self.distance(current, terminal.pose.position) <= self.goal_reached_distance:
                    self.active_goal = None
                    self.terminal_goal = None
                    self.replan_attempt = 0
                    self.replan_cycle_budget.reset()
                    status, goal = ('GOAL_REACHED', ''), None
                elif not self.replan_cycle_budget.try_begin_cycle():
                    status, goal = ('REPLAN_EXHAUSTED', 'cycles={}/{}'.format(
                        self.replan_cycle_budget.used_cycles,
                        self.replan_cycle_budget.max_cycles)), None
                else:
                    self.replan_attempt = 1
                    status = None
                    goal = self.active_goal
                    generation = self.request_generation
        if status is not None:
            self.publish_status(*status)
            return
        self.send_plan(goal, generation, is_replan=True)

    def send_plan(self, goal, generation, is_replan):
        with self.lock:
            odom = self.odom
            if odom is None or generation != self.request_generation or self.planning:
                return
            self.planning = True
        try:
            start = PoseStamped()
            start.header = odom.header
            start.pose = odom.pose.pose
            start = self.transform_pose(start)
        except Exception as exc:
            self.plan_failed(generation, is_replan, 'tf:' + str(exc))
            return
        if not self.client.wait_for_server(rospy.Duration(self.action_wait_timeout)):
            self.plan_failed(generation, is_replan, 'pct_action_unavailable')
            return
        with self.lock:
            # A new user goal may have superseded this request while waiting
            # for the Action server to come up.
            if generation != self.request_generation or not self.planning:
                return
        request = PlanPath3DGoal(start=start, goal=goal)
        self.publish_status('REPLANNING' if is_replan else 'GLOBAL_PLANNING')
        self.client.send_goal(
            request,
            done_cb=lambda state, result: self.plan_done(generation, is_replan, state, result))

    def plan_failed(self, generation, is_replan, reason):
        with self.lock:
            if generation != self.request_generation:
                return
            self.planning = False
            retry = is_replan and self.replan_attempt < self.replan_max_attempts
            if retry:
                delay_index = min(self.replan_attempt - 1, len(self.replan_retry_delays) - 1)
                delay = self.replan_retry_delays[delay_index] if delay_index >= 0 else 0.0
                self.retry_timer = rospy.Timer(
                    rospy.Duration(delay),
                    lambda _event: self.retry_callback(generation), oneshot=True)
        if retry:
            self.publish_status('REPLAN_RETRY', '{}; attempt={} delay={:.1f}s'.format(
                reason, self.replan_attempt + 1, delay))
        else:
            self.publish_status('REPLAN_FAILED' if is_replan else 'ABORTED', reason)

    def retry_callback(self, generation):
        with self.lock:
            self.retry_timer = None
            if (generation != self.request_generation or self.active_goal is None or
                    self.planning):
                return
            self.replan_attempt += 1
            goal = self.active_goal
        self.send_plan(goal, generation, is_replan=True)

    def plan_done(self, generation, is_replan, _state, result):
        with self.lock:
            if generation != self.request_generation:
                return
            self.planning = False
        if result is None:
            self.plan_failed(generation, is_replan, 'no_action_result')
            return
        if result.has_snapped_goal:
            self.validated_goal_pub.publish(result.snapped_goal)
        if result.status != result.SUCCESS:
            self.plan_failed(generation, is_replan, result.message or 'pct_failure')
            return
        try:
            path = self.prepare_path(result.path)
        except ValueError as exc:
            self.plan_failed(generation, is_replan, str(exc))
            return

        terminal = result.snapped_goal if result.has_snapped_goal else path.poses[-1]
        with self.lock:
            if generation != self.request_generation:
                return
            self.terminal_goal = terminal
            self.replan_attempt = 0
            self._cancel_retry_locked()
        self.reference_pub.publish(path)
        self.publish_status('PATH_SWITCHED' if is_replan else 'PATH_TRACKING')

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
        closest = min(range(len(points)),
                      key=lambda i: self.distance(points[i].pose.position, current.pose.position))
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
