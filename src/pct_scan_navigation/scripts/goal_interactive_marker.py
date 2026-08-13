#!/usr/bin/env python3
"""Interactive 3D goal marker with a right-click ``Plan`` action."""

import math

import rospy
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from nav_msgs.msg import Odometry
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class GoalMarker:
    def __init__(self):
        self.frame = rospy.get_param('~navigation_frame', 'map')
        self.odom_topic = rospy.get_param('~body_pose_topic', '/quad_0/body_pose')
        self.goal_topic = rospy.get_param('~goal_topic', '/goal_pose_3d')
        self.validated_goal_topic = rospy.get_param('~validated_goal_topic', '/navigation/validated_goal')
        self.name = rospy.get_param('~marker_name', '3d_goal')
        self.pose = None
        self.server = InteractiveMarkerServer(self.name)
        self.menu = MenuHandler()
        self.plan_id = self.menu.insert('Plan', callback=self.plan_callback)
        self.reset_id = self.menu.insert('Reset to robot', callback=self.reset_callback)
        self.pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(self.validated_goal_topic, PoseStamped, self.validated_goal_callback, queue_size=1)
        self.insert_marker()

    def odom_callback(self, msg):
        if self.pose is None:
            self.pose = msg.pose.pose
            self.insert_marker()

    @staticmethod
    def axis_quaternion(axis):
        """Return a unit quaternion whose local X axis points along axis."""
        half_sqrt = math.sqrt(0.5)
        if axis == 'x':
            return 0.0, 0.0, 0.0, 1.0
        if axis == 'y':
            return 0.0, 0.0, half_sqrt, half_sqrt
        if axis == 'z':
            return 0.0, -half_sqrt, 0.0, half_sqrt
        raise ValueError('unknown marker axis: {}'.format(axis))

    @staticmethod
    def set_control_axis(control, axis):
        qx, qy, qz, qw = GoalMarker.axis_quaternion(axis)
        control.orientation.x = qx
        control.orientation.y = qy
        control.orientation.z = qz
        control.orientation.w = qw

    def validated_goal_callback(self, msg):
        if msg.header.frame_id != self.frame:
            rospy.logwarn('[goal_interactive_marker] Ignore validated goal in frame %s (expected %s)',
                          msg.header.frame_id, self.frame)
            return
        point = msg.pose.position
        if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
            rospy.logwarn('[goal_interactive_marker] Ignore non-finite validated goal')
            return
        self.pose = msg.pose
        self.insert_marker()
        rospy.loginfo('[goal_interactive_marker] Goal snapped to traversable surface: (%.3f, %.3f, %.3f)',
                      point.x, point.y, point.z)

    def insert_marker(self):
        if self.pose is None:
            return
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame
        marker.header.stamp = rospy.Time.now()
        marker.name = self.name
        marker.description = 'Drag goal; right-click and choose Plan'
        marker.scale = 1.2
        marker.pose = self.pose

        visual = InteractiveMarkerControl()
        visual.name = 'menu_visual'
        visual.orientation.w = 1.0
        visual.orientation_mode = InteractiveMarkerControl.INHERIT
        visual.interaction_mode = InteractiveMarkerControl.MENU
        visual.always_visible = True
        body = Marker()
        body.type = Marker.SPHERE
        body.scale.x = body.scale.y = body.scale.z = 0.28
        body.color.r, body.color.g, body.color.b, body.color.a = 0.62, 0.03, 0.02, 1.0
        visual.markers.append(body)

        # A heading arrow is attached to the marker pose, so it rotates with
        # the goal's yaw instead of staying fixed in the world frame.
        heading = Marker()
        heading.type = Marker.ARROW
        heading.scale.x = 0.45
        heading.scale.y = 0.07
        heading.scale.z = 0.07
        heading.color.r, heading.color.g = 0.72, 0.38
        heading.color.b, heading.color.a = 0.02, 1.0
        heading.pose.orientation.w = 1.0
        visual.markers.append(heading)
        marker.controls.append(visual)

        # Explicit X/Y/Z translation axes.  Separate axes are more reliable
        # than MOVE_PLANE across RViz versions.
        for name, mode, axis in (
                ('move_x', InteractiveMarkerControl.MOVE_AXIS, 'x'),
                ('move_y', InteractiveMarkerControl.MOVE_AXIS, 'y'),
                ('move_z', InteractiveMarkerControl.MOVE_AXIS, 'z')):
            control = InteractiveMarkerControl()
            control.name = name
            control.description = name
            control.interaction_mode = mode
            control.orientation_mode = InteractiveMarkerControl.FIXED
            self.set_control_axis(control, axis)
            marker.controls.append(control)

        # Add independent roll, pitch and yaw rings.  In particular, the Z
        # axis ring lets the operator set the final heading of the goal.
        for name, axis in (
                ('rotate_roll', 'x'),
                ('rotate_pitch', 'y'),
                ('rotate_yaw', 'z')):
            control = InteractiveMarkerControl()
            control.name = name
            control.description = name
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            control.orientation_mode = InteractiveMarkerControl.INHERIT
            self.set_control_axis(control, axis)
            marker.controls.append(control)

        # A MENU control is required by RViz to expose MenuHandler entries in
        # the right-click context menu.
        menu_control = InteractiveMarkerControl()
        menu_control.name = 'goal_menu'
        menu_control.description = 'Plan / Reset to robot'
        menu_control.orientation.w = 1.0
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.always_visible = True
        marker.controls.append(menu_control)

        self.server.insert(marker, self.feedback)
        self.menu.apply(self.server, self.name)
        self.server.applyChanges()

    def feedback(self, event):
        if event.marker_name != self.name:
            return
        if event.event_type == event.POSE_UPDATE:
            self.pose = event.pose
            self.insert_marker()

    def reset_callback(self, feedback):
        self.pose = None
        self.server.erase(self.name)
        self.server.applyChanges()
        rospy.sleep(0.05)

    def plan_callback(self, feedback):
        if self.pose is None:
            return
        goal = PoseStamped()
        goal.header.frame_id = self.frame
        goal.header.stamp = rospy.Time.now()
        goal.pose = self.pose
        self.pub.publish(goal)
        rospy.loginfo('[goal_interactive_marker] Plan: (%.3f, %.3f, %.3f)',
                      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)


if __name__ == '__main__':
    rospy.init_node('goal_interactive_marker')
    GoalMarker()
    rospy.spin()
