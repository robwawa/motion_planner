"""Projection of authoritative dynamic-obstacle snapshots onto a PCT map."""

import math
import threading

import numpy as np


class DynamicObstacleLayer:
    """Maintain dynamic PCT costs from dynamic_perception_3d snapshots only."""

    def __init__(self, traversability, ground, ceiling, center, resolution,
                 traversable_threshold, lethal_cost=100,
                 terrain_ignore_height=0.08, collision_top=0.56,
                 layer_height_tolerance=0.25,
                 robot_radius=0.32, safety_margin=0.15, inflation_radius=0.50,
                 max_points_per_snapshot=200000, assignment_chunk_size=20000,
                 max_dynamic_memory_mb=128.0):
        self.trav = np.asarray(traversability)
        self.ground = np.asarray(ground)
        self.ceiling = np.asarray(ceiling)
        if self.trav.shape != self.ground.shape or self.trav.shape != self.ceiling.shape:
            raise ValueError('PCT traversability, ground and ceiling shapes must match')
        if self.trav.ndim != 3:
            raise ValueError('PCT arrays must have [layer,row,col] layout')
        self.shape = self.trav.shape
        self.layers, self.rows, self.cols = self.shape
        self.center = np.asarray(center, dtype=np.float64)
        self.resolution = float(resolution)
        self.threshold = float(traversable_threshold)
        self.lethal_cost = int(lethal_cost)
        if not 0 < self.lethal_cost <= 255 or self.lethal_cost <= self.threshold:
            raise ValueError('dynamic lethal cost must be uint8 and exceed PCT threshold')
        self.terrain_ignore_height = float(terrain_ignore_height)
        self.collision_top = float(collision_top)
        self.layer_height_tolerance = float(layer_height_tolerance)
        self.robot_radius = float(robot_radius)
        self.safety_margin = float(safety_margin)
        self.inflation_radius = float(inflation_radius)
        if self.inflation_radius < self.robot_radius + self.safety_margin:
            raise ValueError('inflation radius must cover robot radius and safety margin')
        self.max_points_per_snapshot = max(1, int(max_points_per_snapshot))
        self.assignment_chunk_size = max(1, int(assignment_chunk_size))
        # Keep obstacle height with occupancy so each complete source snapshot
        # can be projected onto the matching PCT layer.
        required_bytes = int(np.prod(self.shape, dtype=np.int64)) * 6
        if required_bytes > float(max_dynamic_memory_mb) * 1024.0 * 1024.0:
            raise MemoryError('dynamic layer needs {:.1f} MiB, budget is {:.1f} MiB'.format(
                required_bytes / (1024.0 * 1024.0), max_dynamic_memory_mb))
        self.occupied = np.zeros(self.shape, dtype=bool)
        self.occupied_z = np.full(self.shape, np.nan, dtype=np.float32)
        self.dynamic_cost = np.zeros(self.shape, dtype=np.uint8)
        self.version = 0
        self.last_assigned_points = np.empty((0, 4), dtype=np.float32)
        self.lock = threading.RLock()
        self._offsets = self._make_inflation_offsets()

    def _make_inflation_offsets(self):
        radius_cells = int(math.ceil(self.inflation_radius / self.resolution))
        inscribed = self.robot_radius + self.safety_margin
        sigma = max((self.inflation_radius - inscribed) / 2.0, self.resolution)
        offsets = []
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                distance = math.hypot(dr, dc) * self.resolution
                if distance > self.inflation_radius:
                    continue
                if distance <= inscribed:
                    cost = self.lethal_cost
                else:
                    cost = max(1, min(self.lethal_cost - 1, int(round(
                        (self.threshold - 1) * math.exp(
                            -0.5 * ((distance - inscribed) / sigma) ** 2)))))
                offsets.append((dr, dc, np.uint8(cost)))
        return offsets

    def _state_equals(self, occupied, occupied_z):
        """Compare state while treating two NaN height sentinels as equal.

        Avoid ``np.array_equal(..., equal_nan=True)`` because deployment on
        ROS Noetic may use an older NumPy without that keyword.
        """
        return (np.array_equal(occupied, self.occupied) and
                np.all((occupied_z == self.occupied_z) |
                       (np.isnan(occupied_z) & np.isnan(self.occupied_z))))

    def world_to_grid(self, xy):
        points = np.asarray(xy, dtype=np.float64)
        relative = points - self.center
        base = np.rint(relative / self.resolution).astype(np.int64)
        base[..., 0] += self.rows // 2
        base[..., 1] += self.cols // 2
        return base[..., 0], base[..., 1]

    def grid_to_world(self, rows, cols):
        return ((np.asarray(rows) - self.rows // 2) * self.resolution + self.center[0],
                (np.asarray(cols) - self.cols // 2) * self.resolution + self.center[1])

    def _assign_point_chunk(self, points):
        empty_indices = np.empty(0, dtype=np.int64)
        empty_points = np.empty((0, 3), dtype=np.float32)
        rows, cols = self.world_to_grid(points[:, :2])
        inside = ((rows >= 0) & (rows < self.rows) & (cols >= 0) & (cols < self.cols))
        points, rows, cols = points[inside], rows[inside], cols[inside]
        if len(points) == 0:
            return empty_indices, empty_indices, empty_indices, empty_points
        ground = self.ground[:, rows, cols]
        ceiling = self.ceiling[:, rows, cols]
        trav = self.trav[:, rows, cols]
        dz = points[None, :, 2] - ground
        valid = (np.isfinite(ground) & np.isfinite(ceiling) & (trav <= self.threshold) &
                 (dz > self.terrain_ignore_height) & (dz <= self.collision_top) &
                 (points[None, :, 2] >= ground - self.layer_height_tolerance) &
                 (points[None, :, 2] <= ceiling + self.layer_height_tolerance))
        score = np.where(valid, np.abs(dz), np.inf)
        selected = np.argmin(score, axis=0)
        keep = np.isfinite(score[selected, np.arange(score.shape[1])])
        return selected[keep], rows[keep], cols[keep], points[keep]

    def assign_points(self, points):
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError('points must be an Nx3 array')
        points = points[:, :3]
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) > self.max_points_per_snapshot:
            stride = int(math.ceil(len(points) / float(self.max_points_per_snapshot)))
            points = points[::stride]
        chunks = [self._assign_point_chunk(points[start:start + self.assignment_chunk_size])
                  for start in range(0, len(points), self.assignment_chunk_size)]
        chunks = [chunk for chunk in chunks if len(chunk[0])]
        if not chunks:
            return np.empty((0, 4), dtype=np.float32)
        selected = np.concatenate([chunk[0] for chunk in chunks])
        rows = np.concatenate([chunk[1] for chunk in chunks])
        cols = np.concatenate([chunk[2] for chunk in chunks])
        kept_points = np.concatenate([chunk[3] for chunk in chunks])
        flat = (selected * self.rows + rows) * self.cols + cols
        _, unique = np.unique(flat, return_index=True)
        return np.column_stack((selected[unique], rows[unique], cols[unique],
                                kept_points[unique, 2])).astype(np.float32)

    def replace_snapshot(self, points):
        """Replace PCT occupancy with one complete lifecycle-managed snapshot."""
        assigned = self.assign_points(points)
        next_occupied = np.zeros(self.shape, dtype=bool)
        next_z = np.full(self.shape, np.nan, dtype=np.float32)
        if len(assigned):
            next_occupied[assigned[:, 0].astype(np.intp), assigned[:, 1].astype(np.intp),
                          assigned[:, 2].astype(np.intp)] = True
            next_z[assigned[:, 0].astype(np.intp), assigned[:, 1].astype(np.intp),
                   assigned[:, 2].astype(np.intp)] = assigned[:, 3]
        with self.lock:
            self.last_assigned_points = assigned
            if self._state_equals(next_occupied, next_z):
                return False
            self.occupied = next_occupied
            self.occupied_z = next_z
            self._rebuild_cost_locked()
            return True

    def clear(self):
        with self.lock:
            if not np.any(self.occupied) and not np.any(self.dynamic_cost):
                return False
            self.occupied.fill(False)
            self.occupied_z.fill(np.nan)
            self.dynamic_cost.fill(0)
            self.last_assigned_points = np.empty((0, 4), dtype=np.float32)
            self.version += 1
            return True

    def _rebuild_cost_locked(self):
        self.dynamic_cost.fill(0)
        for layer in range(self.layers):
            rows, cols = np.nonzero(self.occupied[layer])
            for dr, dc, cost in self._offsets:
                rr, cc = rows + dr, cols + dc
                valid = ((rr >= 0) & (rr < self.rows) & (cc >= 0) & (cc < self.cols))
                if np.any(valid):
                    target = self.dynamic_cost[layer, rr[valid], cc[valid]]
                    self.dynamic_cost[layer, rr[valid], cc[valid]] = np.maximum(target, cost)
        self.version += 1

    def snapshot(self):
        with self.lock:
            return self.dynamic_cost, int(self.version)

    def nonzero_cells(self, max_points=None):
        with self.lock:
            flat = np.flatnonzero(self.dynamic_cost)
            if max_points and len(flat) > max_points:
                flat = flat[::int(math.ceil(len(flat) / float(max_points)))]
            layers, rows, cols = np.unravel_index(flat, self.shape)
            return layers, rows, cols, self.dynamic_cost[layers, rows, cols]
