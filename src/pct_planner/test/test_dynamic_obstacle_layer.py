#!/usr/bin/env python3

import os
import sys
import unittest

import numpy as np

SCRIPT_DIR = os.path.join(os.path.dirname(__file__), '..', 'planner', 'scripts')
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

from dynamic_obstacle_layer import DynamicObstacleLayer


class DynamicObstacleLayerTest(unittest.TestCase):
    def make_layer(self, **overrides):
        shape = (2, 11, 11)
        traversability = np.zeros(shape, dtype=np.float32)
        ground = np.zeros(shape, dtype=np.float32)
        ground[1] = 3.0
        ceiling = np.full(shape, 2.8, dtype=np.float32)
        ceiling[1] = 6.0
        kwargs = dict(
            traversability=traversability, ground=ground, ceiling=ceiling,
            center=(0.0, 0.0), resolution=0.1, traversable_threshold=49.0,
            lethal_cost=100, terrain_ignore_height=0.08, collision_top=0.8,
            robot_radius=0.1, safety_margin=0.05, inflation_radius=0.2,
            max_points_per_snapshot=100)
        kwargs.update(overrides)
        return DynamicObstacleLayer(**kwargs)

    def test_same_xy_points_project_to_independent_floors(self):
        layer = self.make_layer()
        assigned = layer.assign_points(np.asarray([
            [0.0, 0.0, 0.5], [0.0, 0.0, 3.5]], dtype=np.float32))
        self.assertEqual({0, 1}, set(assigned[:, 0].astype(int)))

    def test_ground_and_out_of_bounds_points_are_not_projected(self):
        layer = self.make_layer()
        assigned = layer.assign_points(np.asarray([
            [0.0, 0.0, 0.05], [99.0, 99.0, 0.5]], dtype=np.float32))
        self.assertEqual(0, len(assigned))

    def test_snapshot_replaces_previous_dynamic_occupancy(self):
        layer = self.make_layer()
        first = np.asarray([[0.0, 0.0, 0.5]], dtype=np.float32)
        second = np.asarray([[0.3, 0.0, 0.5]], dtype=np.float32)
        self.assertTrue(layer.replace_snapshot(first))
        first_row, first_col = layer.world_to_grid(np.asarray([0.0, 0.0]))
        self.assertTrue(layer.occupied[0, first_row, first_col])
        self.assertTrue(layer.replace_snapshot(second))
        second_row, second_col = layer.world_to_grid(np.asarray([0.3, 0.0]))
        self.assertFalse(layer.occupied[0, first_row, first_col])
        self.assertTrue(layer.occupied[0, second_row, second_col])
        self.assertGreater(np.count_nonzero(layer.dynamic_cost), 1)

    def test_empty_snapshot_clears_dynamic_layer_immediately(self):
        layer = self.make_layer()
        layer.replace_snapshot(np.asarray([[0.0, 0.0, 0.5]], dtype=np.float32))
        self.assertTrue(layer.replace_snapshot(np.empty((0, 3), dtype=np.float32)))
        self.assertEqual(0, np.count_nonzero(layer.dynamic_cost))

    def test_shape_mismatch_is_rejected(self):
        with self.assertRaises(ValueError):
            DynamicObstacleLayer(
                np.zeros((1, 2, 2), dtype=np.float32),
                np.zeros((1, 2, 2), dtype=np.float32),
                np.zeros((1, 2, 3), dtype=np.float32),
                (0.0, 0.0), 0.1, 49.0)

    def test_assignment_is_stable_across_small_memory_chunks(self):
        points = np.asarray([
            [0.0, 0.0, 0.5], [0.1, 0.0, 0.5],
            [0.0, 0.1, 3.5], [0.1, 0.1, 3.5]], dtype=np.float32)
        unchunked = self.make_layer(assignment_chunk_size=100).assign_points(points)
        chunked = self.make_layer(assignment_chunk_size=1).assign_points(points)
        np.testing.assert_array_equal(unchunked, chunked)

    def test_memory_budget_is_checked_before_allocation(self):
        with self.assertRaises(MemoryError):
            self.make_layer(max_dynamic_memory_mb=0.0001)


if __name__ == '__main__':
    unittest.main()
