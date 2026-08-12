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
        visual.interaction_mode = InteractiveMarkerControl.MENU
        visual.always_visible = True
        body = Marker()
        body.type = Marker.SPHERE
        body.scale.x = body.scale.y = body.scale.z = 0.28
        body.color.r, body.color.g, body.color.b, body.color.a = 1.0, 0.2, 0.1, 0.95
        visual.markers.append(body)
        marker.controls.append(visual)

        # Explicit X/Y/Z translation axes, plus yaw rotation.  Separate axes
        # are more reliable than MOVE_PLANE across RViz versions.
        for name, mode, orientation in (
                ('move_x', InteractiveMarkerControl.MOVE_AXIS, (1.0, 0.0, 0.0, 1.0)),
                ('move_y', InteractiveMarkerControl.MOVE_AXIS, (0.0, 1.0, 0.0, 1.0)),
                ('move_z', InteractiveMarkerControl.MOVE_AXIS, (0.0, 0.0, 1.0, 1.0)),
                ('rotate_yaw', InteractiveMarkerControl.ROTATE_AXIS, (0.0, 0.0, 1.0, 1.0))):
            control = InteractiveMarkerControl()
            control.name = name
            control.description = name
            control.interaction_mode = mode
            norm = math.sqrt(sum(value * value for value in orientation))
            qx, qy, qz, qw = (value / norm for value in orientation)
            control.orientation.w, control.orientation.x = qw, qx
            control.orientation.y, control.orientation.z = qy, qz
            marker.controls.append(control)

        # A MENU control is required by RViz to expose MenuHandler entries in
        # the right-click context menu.
        menu_control = InteractiveMarkerControl()
        menu_control.name = 'goal_menu'
        menu_control.description = 'Plan / Reset to robot'
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
