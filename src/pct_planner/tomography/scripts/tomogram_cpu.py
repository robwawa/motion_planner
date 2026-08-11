"""NumPy/SciPy implementation of PCT tomogram construction.

This mirrors the three CUDA stages in :mod:`tomogram` while keeping the same
public methods and return values.  It is intended for hosts without a CUDA
runtime; CUDA remains the preferred backend for large point clouds.
"""

import os
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
from scipy import ndimage


def _round_like_cuda(values):
    """Match C/CUDA ``round`` for positive and negative half values."""
    return np.where(values >= 0.0, np.floor(values + 0.5), np.ceil(values - 0.5))


class CpuTomogram(object):
    """CPU backend with the same API as the original CuPy ``Tomogram``."""

    backend = 'cpu'

    def __init__(self, cfg):
        self.resolution = cfg.map.resolution
        self.slice_dh = cfg.map.slice_dh
        self.half_trav_k_size = int(cfg.trav.kernel_size / 2)
        self.interval_min = cfg.trav.interval_min
        self.interval_free = cfg.trav.interval_free
        self.step_stand = 1.2 * self.resolution * np.tan(cfg.trav.slope_max)
        self.step_cross = cfg.trav.step_max
        self.standable_th = int(
            cfg.trav.standable_ratio * (2 * self.half_trav_k_size + 1) ** 2
        ) - 1
        self.cost_barrier = float(cfg.trav.cost_barrier)
        self.safe_margin = cfg.trav.safe_margin
        self.inflation = cfg.trav.inflation
        self.half_inf_k_size = int(
            (self.safe_margin + self.inflation) / self.resolution
        )

    def initMappingEnv(self, center, map_dim_x, map_dim_y, n_slice_init, slice_h0):
        self.center = np.asarray(center, dtype=np.float32)
        self.map_dim_x = int(map_dim_x)
        self.map_dim_y = int(map_dim_y)
        self.n_slice_init = int(n_slice_init)
        self.slice_h0 = float(slice_h0)
        self.slice_heights = (
            np.float32(self.slice_h0) +
            np.arange(self.n_slice_init, dtype=np.float32) * np.float32(self.slice_dh)
        )
        self.neighbourhood_1d = np.ones(
            2 * self.half_trav_k_size + 1, dtype=np.int16
        )
        self.cpu_workers = min(
            8, max(1, os.cpu_count() or 1), max(1, self.n_slice_init)
        )

        axis = (
            np.arange(
                -self.half_inf_k_size,
                self.half_inf_k_size + 1,
                dtype=np.float32,
            ) * self.resolution
        )
        distance = np.hypot(axis[:, None], axis[None, :])
        weights = np.clip(
            1.0 -
            (distance - self.inflation) / (self.safe_margin + self.resolution),
            0.0,
            1.0,
        ).astype(np.float32)
        self.inf_footprint = weights > 0.0
        self.inf_log_structure = np.full(weights.shape, -np.inf, dtype=np.float32)
        self.inf_log_structure[self.inf_footprint] = np.log(
            weights[self.inf_footprint]
        )

    def _build_layers(self, points):
        relative = (points[:, :2] - self.center) / self.resolution
        xy = _round_like_cuda(relative).astype(np.int32)
        xy[:, 0] += self.map_dim_x // 2
        xy[:, 1] += self.map_dim_y // 2
        inside = (
            (xy[:, 0] >= 0) & (xy[:, 0] < self.map_dim_x) &
            (xy[:, 1] >= 0) & (xy[:, 1] < self.map_dim_y)
        )
        xy = xy[inside]
        z = points[inside, 2]
        flat_xy = xy[:, 0] * self.map_dim_y + xy[:, 1]
        layer_size = self.map_dim_x * self.map_dim_y

        # A point becomes ground at the first slice whose height is greater
        # than or equal to its Z coordinate.  Bucket each point once, then use
        # cumulative reductions to reproduce its contribution to all slices.
        first_ground = np.searchsorted(self.slice_heights, z, side='left')

        layers_g = np.full((self.n_slice_init, layer_size), -1e6, dtype=np.float32)
        ground_mask = first_ground < self.n_slice_init
        np.maximum.at(
            layers_g,
            (first_ground[ground_mask], flat_xy[ground_mask]),
            z[ground_mask],
        )
        np.maximum.accumulate(layers_g, axis=0, out=layers_g)

        layers_c = np.full((self.n_slice_init, layer_size), 1e6, dtype=np.float32)
        ceiling_mask = first_ground > 0
        last_ceiling = np.minimum(
            first_ground[ceiling_mask] - 1, self.n_slice_init - 1
        )
        np.minimum.at(
            layers_c,
            (last_ceiling, flat_xy[ceiling_mask]),
            z[ceiling_mask],
        )
        reversed_ceiling = layers_c[::-1]
        np.minimum.accumulate(
            reversed_ceiling, axis=0, out=reversed_ceiling
        )

        return (
            layers_g.reshape(self.n_slice_init, self.map_dim_x, self.map_dim_y),
            layers_c.reshape(self.n_slice_init, self.map_dim_x, self.map_dim_y),
        )

    def _traversability(self, layers_g, layers_c, grad_mag_sq, grad_mag_max):
        interval = layers_c - layers_g
        trav_cost = np.maximum(0.0, 20.0 * (self.interval_free - interval)).astype(
            np.float32
        )
        barrier = interval < self.interval_min
        step_stand_sq = self.step_stand ** 2
        step_cross_sq = self.step_cross ** 2
        standable = grad_mag_sq <= step_stand_sq
        crossable = grad_mag_max <= step_cross_sq

        # CUDA counts only gradients strictly below the standing threshold.
        standable_count = ndimage.convolve1d(
            (grad_mag_sq < step_stand_sq).astype(np.int16),
            self.neighbourhood_1d,
            axis=1,
            mode='constant',
            cval=0,
        )
        standable_count = ndimage.convolve1d(
            standable_count,
            self.neighbourhood_1d,
            axis=2,
            mode='constant',
            cval=0,
        )
        crossing = ~standable
        barrier |= crossing & (~crossable | (standable_count < self.standable_th))
        if step_stand_sq > 0.0:
            trav_cost[standable] += 15.0 * grad_mag_sq[standable] / step_stand_sq
        if step_cross_sq > 0.0:
            accepted_crossing = crossing & ~barrier
            trav_cost[accepted_crossing] += (
                20.0 * grad_mag_max[accepted_crossing] / step_cross_sq
            )
        trav_cost[barrier] = self.cost_barrier

        inflated_cost = np.empty_like(trav_cost)

        def inflate_layer(layer):
            log_cost = np.full(trav_cost[layer].shape, -np.inf, dtype=np.float32)
            positive = trav_cost[layer] > 0.0
            log_cost[positive] = np.log(trav_cost[layer][positive])
            dilated = ndimage.grey_dilation(
                log_cost,
                footprint=self.inf_footprint,
                structure=self.inf_log_structure,
                mode='constant',
                cval=-np.inf,
            )
            np.exp(dilated, out=inflated_cost[layer])

        if self.cpu_workers == 1:
            for layer in range(self.n_slice_init):
                inflate_layer(layer)
        else:
            with ThreadPoolExecutor(max_workers=self.cpu_workers) as executor:
                list(executor.map(inflate_layer, range(self.n_slice_init)))
        return inflated_cost

    def point2map(self, points):
        points = np.asarray(points, dtype=np.float32)
        points = points[~np.isnan(points).any(axis=1)]

        started = time.perf_counter()
        layers_g, layers_c = self._build_layers(points)
        grad_mag_sq = np.zeros_like(layers_g)
        grad_mag_max = np.zeros_like(layers_g)
        diff_x_sq = np.maximum(
            (layers_g[:, 1:-1, :] - layers_g[:, :-2, :]) ** 2,
            (layers_g[:, 1:-1, :] - layers_g[:, 2:, :]) ** 2,
        )
        diff_y_sq = np.maximum(
            (layers_g[:, :, 1:-1] - layers_g[:, :, :-2]) ** 2,
            (layers_g[:, :, 1:-1] - layers_g[:, :, 2:]) ** 2,
        )
        grad_mag_sq[:, 1:-1, 1:-1] = (
            diff_x_sq[:, :, 1:-1] + diff_y_sq[:, 1:-1, :]
        )
        grad_mag_max[:, 1:-1, 1:-1] = np.maximum(
            diff_x_sq[:, :, 1:-1], diff_y_sq[:, 1:-1, :]
        )
        t_map = (time.perf_counter() - started) * 1e3

        started = time.perf_counter()
        inflated_cost = self._traversability(
            layers_g, layers_c, grad_mag_sq, grad_mag_max
        )
        t_trav = (time.perf_counter() - started) * 1e3

        started = time.perf_counter()
        idx_simp = [0]
        if self.n_slice_init > 1:
            lower_idx, middle_idx = 0, 1
            diff_h = layers_g[1:] - layers_g[:-1]
            while middle_idx < self.n_slice_init - 2:
                unique = (
                    ((layers_g[middle_idx] - layers_g[lower_idx] > 0) |
                     (inflated_cost[lower_idx] > inflated_cost[middle_idx])) &
                    (diff_h[middle_idx] > 0) &
                    (inflated_cost[middle_idx] < self.cost_barrier)
                )
                if np.any(unique):
                    idx_simp.append(middle_idx)
                    lower_idx = middle_idx
                middle_idx += 1
            idx_simp.append(middle_idx)

        layers_t = inflated_cost[idx_simp]
        selected_g = layers_g[idx_simp]
        selected_c = layers_c[idx_simp]
        trav_gx = np.zeros_like(selected_g)
        trav_gy = np.zeros_like(selected_g)
        trav_gx[:, 1:-1, :] = layers_t[:, 2:, :] - layers_t[:, :-2, :]
        trav_gy[:, :, 1:-1] = layers_t[:, :, 2:] - layers_t[:, :, :-2]
        layers_g = np.where(selected_g > -1e6, selected_g, np.nan)
        layers_c = np.where(selected_c < 1e6, selected_c, np.nan)
        t_simp = (time.perf_counter() - started) * 1e3

        return layers_t, trav_gx, trav_gy, layers_g, layers_c, {
            't_map': t_map,
            't_trav': t_trav,
            't_simp': t_simp,
        }
