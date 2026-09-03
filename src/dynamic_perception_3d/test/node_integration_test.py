#!/usr/bin/env python3

import math
import threading
import unittest

import rospy
import rostest
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header
from visualization_msgs.msg import Marker


class DynamicPerceptionNodeIntegration(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.outputs = []
        self.health = []
        self.current_lidar = []
        self.history_windows = []
        self.preprocess_windows = []
        self.publisher = rospy.Publisher('/test/lidar', PointCloud2, queue_size=1)
        self.subscriber = rospy.Subscriber(
            '/dynamic_perception/dynamic_cloud', PointCloud2, self._output, queue_size=5)
        self.health_subscriber = rospy.Subscriber(
            '/dynamic_perception/scan_healthy', Bool, self._health, queue_size=5)
        self.current_lidar_subscriber = rospy.Subscriber(
            '/dynamic_perception/current_lidar', PointCloud2,
            self._current_lidar, queue_size=5)
        self.history_window_subscriber = rospy.Subscriber(
            '/dynamic_perception/history_window', Marker,
            self._history_window, queue_size=5)
        self.preprocess_window_subscriber = rospy.Subscriber(
            '/dynamic_perception/preprocess_window', Marker,
            self._preprocess_window, queue_size=5)

    def _output(self, message):
        with self.lock:
            self.outputs.append(message)

    def _health(self, message):
        with self.lock:
            self.health.append(message.data)

    def _current_lidar(self, message):
        with self.lock:
            self.current_lidar.append(message)

    def _history_window(self, message):
        with self.lock:
            self.history_windows.append(message)

    def _preprocess_window(self, message):
        with self.lock:
            self.preprocess_windows.append(message)

    def test_map_frame_input_is_confirmed_after_two_hits(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while self.publisher.get_num_connections() == 0 and rospy.Time.now() < deadline:
            rospy.sleep(0.05)
        self.assertGreater(self.publisher.get_num_connections(), 0)

        points = [
            (2.0, 0.0, 0.0),
            (2.0, 0.1, 0.0),
            (1.0, 1.00, 0.2),
            (1.0, 1.03, 0.2),
            (1.0, 1.06, 0.2),
            (1.0, 1.09, 0.2),
        ]
        for _ in range(4):
            header = Header(stamp=rospy.Time.now(), frame_id='map')
            self.publisher.publish(pc2.create_cloud_xyz32(header, points))
            rospy.sleep(0.15)

        deadline = rospy.Time.now() + rospy.Duration(3.0)
        confirmed = None
        while rospy.Time.now() < deadline:
            with self.lock:
                nonempty = [msg for msg in self.outputs if msg.width * msg.height > 0]
            if nonempty:
                confirmed = nonempty[-1]
                break
            rospy.sleep(0.05)
        self.assertIsNotNone(confirmed)
        self.assertEqual('map', confirmed.header.frame_id)
        self.assertGreaterEqual(confirmed.width * confirmed.height, 3)

        deadline = rospy.Time.now() + rospy.Duration(3.0)
        while rospy.Time.now() < deadline:
            with self.lock:
                have_windows = bool(self.history_windows and self.preprocess_windows)
            if have_windows:
                break
            rospy.sleep(0.05)
        with self.lock:
            history_window = self.history_windows[-1]
            preprocess_window = self.preprocess_windows[-1]
        self.assertEqual('map', history_window.header.frame_id)
        self.assertEqual('map', preprocess_window.header.frame_id)
        self.assertEqual(Marker.LINE_LIST, history_window.type)
        self.assertEqual(Marker.LINE_LIST, preprocess_window.type)
        self.assertEqual(24, len(history_window.points))
        self.assertGreater(len(preprocess_window.points), 24)
        self.assertAlmostEqual(-0.25, min(point.z for point in history_window.points),
                               places=3)
        self.assertAlmostEqual(1.25, max(point.z for point in history_window.points),
                               places=3)
        envelope_ranges = [math.sqrt((point.x - 1.0) ** 2 + point.y ** 2 + point.z ** 2)
                           for point in preprocess_window.points]
        self.assertAlmostEqual(0.1, min(envelope_ranges), places=3)
        self.assertAlmostEqual(10.0, max(envelope_ranges), places=3)

        with self.lock:
            output_count = len(self.outputs)
            health_count = len(self.health)
        empty_header = Header(stamp=rospy.Time.now(), frame_id='map')
        self.publisher.publish(pc2.create_cloud_xyz32(empty_header, []))
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        retained = False
        unhealthy = False
        while rospy.Time.now() < deadline:
            with self.lock:
                new_outputs = self.outputs[output_count:]
                retained = any(msg.width * msg.height > 0 for msg in new_outputs)
                unhealthy = False in self.health[health_count:]
            if retained and unhealthy:
                break
            rospy.sleep(0.05)
        self.assertTrue(retained)
        self.assertTrue(unhealthy)

    def test_base_window_accepts_point_outside_sensor_centered_square(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while self.publisher.get_num_connections() == 0 and rospy.Time.now() < deadline:
            rospy.sleep(0.05)
        self.assertGreater(self.publisher.get_num_connections(), 0)

        # The test TF places the LiDAR 1 m ahead of base.  These points are
        # inside the base-local 5 m window, but would be outside the former
        # sensor-centered +/-5 m XY crop.
        points = [
            (-4.8, 0.00, 0.2),
            (-4.8, 0.06, 0.2),
            (-4.8, 0.12, 0.2),
        ]
        with self.lock:
            initial_count = len(self.current_lidar)
        header = Header(stamp=rospy.Time.now(), frame_id='map')
        self.publisher.publish(pc2.create_cloud_xyz32(header, points))

        deadline = rospy.Time.now() + rospy.Duration(3.0)
        observed = False
        while rospy.Time.now() < deadline:
            with self.lock:
                messages = list(self.current_lidar[initial_count:])
            for message in messages:
                cloud_points = list(pc2.read_points(
                    message, field_names=('x', 'y', 'z'), skip_nans=True))
                if any(point[0] < -4.5 for point in cloud_points):
                    observed = True
                    break
            if observed:
                break
            rospy.sleep(0.05)
        self.assertTrue(observed)

    def test_sensor_height_outside_limits_cannot_supply_visibility_evidence(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while self.publisher.get_num_connections() == 0 and rospy.Time.now() < deadline:
            rospy.sleep(0.05)
        self.assertGreater(self.publisher.get_num_connections(), 0)

        # These points satisfy the wide test FOV, but exceed the default
        # sensor-frame max_z=3.0 and therefore leave visibility_scan empty.
        points = [(2.0, 0.00, 4.0), (2.0, 0.06, 4.0), (2.0, 0.12, 4.0)]
        with self.lock:
            initial_count = len(self.health)
        header = Header(stamp=rospy.Time.now(), frame_id='map')
        self.publisher.publish(pc2.create_cloud_xyz32(header, points))

        deadline = rospy.Time.now() + rospy.Duration(3.0)
        unhealthy = False
        while rospy.Time.now() < deadline:
            with self.lock:
                unhealthy = False in self.health[initial_count:]
            if unhealthy:
                break
            rospy.sleep(0.05)
        self.assertTrue(unhealthy)

    def test_unrelated_returns_clear_removed_obstacle_directly(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while self.publisher.get_num_connections() == 0 and rospy.Time.now() < deadline:
            rospy.sleep(0.05)
        self.assertGreater(self.publisher.get_num_connections(), 0)

        # Confirm an obstacle, then publish only unrelated valid returns. The
        # points are above marking_height, so they keep the visibility scan
        # healthy but cannot create a replacement dynamic observation.
        obstacle = [(1.5, 0.10, 1.50), (1.5, 0.16, 1.50), (1.5, 0.22, 1.50)]
        for _ in range(2):
            header = Header(stamp=rospy.Time.now(), frame_id='map')
            self.publisher.publish(pc2.create_cloud_xyz32(header, obstacle))
            rospy.sleep(0.15)

        with self.lock:
            output_count = len(self.outputs)
        background_returns = [
            (1.8, 0.16, 2.40), (1.8, 0.256, 2.40), (1.8, 0.352, 2.40),
        ]
        header = Header(stamp=rospy.Time.now(), frame_id='map')
        self.publisher.publish(pc2.create_cloud_xyz32(header, background_returns))

        deadline = rospy.Time.now() + rospy.Duration(3.0)
        cleared = False
        while rospy.Time.now() < deadline:
            with self.lock:
                messages = list(self.outputs[output_count:])
            for message in messages:
                cloud_points = list(pc2.read_points(
                    message, field_names=('x', 'y', 'z'), skip_nans=True))
                old_obstacle_present = any(
                    1.3 <= point[0] <= 1.7 and -0.1 <= point[1] <= 0.4
                    for point in cloud_points)
                if not old_obstacle_present:
                    cleared = True
                    break
            if cleared:
                break
            rospy.sleep(0.05)
        self.assertTrue(cleared)


if __name__ == '__main__':
    rospy.init_node('dynamic_perception_3d_node_integration_test')
    rostest.rosrun(
        'dynamic_perception_3d', 'dynamic_perception_3d_node_integration',
        DynamicPerceptionNodeIntegration)
