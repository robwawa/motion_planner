#!/usr/bin/env python3
"""Transform the simulated Mid360 cloud into the shared map frame."""

import threading

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud, PointCloud2


class PointCloudBridge:
    def __init__(self):
        self.lock = threading.Lock()
        self.odom = None
        self.listener = tf.TransformListener()
        self.blind = rospy.get_param("~laser_blind", 0.5)
        self.min_angle = np.deg2rad(rospy.get_param("~min_angle", -7.0))
        self.max_angle = np.deg2rad(rospy.get_param("~max_angle", 55.0))
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.sensor_frame = rospy.get_param("~sensor_frame", "laser_livox")
        odom_topic = rospy.get_param("~odom_topic", "/Odometry_gazebo")
        cloud_topic = rospy.get_param("~cloud_topic", "/livox/Pointcloud2")
        raw_cloud_topic = rospy.get_param("~raw_cloud_topic", "/scan")
        self.publisher = rospy.Publisher(cloud_topic, PointCloud2, queue_size=2)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=5)
        rospy.Subscriber(raw_cloud_topic, PointCloud, self.cloud_callback, queue_size=2)
        rospy.loginfo("Mid360 bridge: %s -> %s (%s)", raw_cloud_topic, cloud_topic, self.map_frame)

    def odom_callback(self, message):
        with self.lock:
            self.odom = message

    @staticmethod
    def rotation_matrix(quaternion):
        return tf.transformations.quaternion_matrix(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])[:3, :3]

    def cloud_callback(self, message):
        with self.lock:
            odom = self.odom
        if odom is None or not message.points:
            return

        points = np.asarray([[p.x, p.y, p.z] for p in message.points], dtype=np.float32)
        distance = np.linalg.norm(points, axis=1)
        vertical_angle = np.arctan2(points[:, 2], np.linalg.norm(points[:, :2], axis=1))
        mask = (distance >= self.blind) & (vertical_angle >= self.min_angle) & (vertical_angle <= self.max_angle)
        points = points[mask]
        if points.size == 0:
            return

        try:
            translation, rotation = self.listener.lookupTransform(
                self.base_frame, self.sensor_frame, rospy.Time(0))
            points = (tf.transformations.quaternion_matrix(rotation)[:3, :3] @ points.T).T
            points += np.asarray(translation)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as error:
            rospy.logwarn_throttle(2.0, "Mid360 extrinsic unavailable: %s", error)
            return

        pose = odom.pose.pose
        points = (self.rotation_matrix(pose.orientation) @ points.T).T
        points += np.asarray([pose.position.x, pose.position.y, pose.position.z])

        header = message.header
        header.frame_id = self.map_frame
        self.publisher.publish(pc2.create_cloud_xyz32(header, points.tolist()))


if __name__ == "__main__":
    rospy.init_node("pct_scan_pointcloud_bridge")
    PointCloudBridge()
    rospy.spin()
