import os
import sys
import pickle
import numpy as np

from utils import *
from pct_terrain_layout import pos2idx as terrain_pos2idx, select_layer as terrain_select_layer

sys.path.append('../')
from lib import a_star, ele_planner, traj_opt

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class TomogramPlanner(object):
    def __init__(self, cfg):
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate

        self.tomo_dir = rsg_root + self.cfg.wrapper.tomo_dir

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None
        self.trav = None
        self.elev_g = None
        self.traversable_cost_threshold = 20.0

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)

    def loadTomogram(self, tomo_file):
        with open(self.tomo_dir + tomo_file + '.pickle', 'rb') as handle:
            data_dict = pickle.load(handle)

            tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        elev_g_raw = tomogram[3]
        self.elev_g_valid = np.isfinite(elev_g_raw)
        elev_g = np.nan_to_num(elev_g_raw, nan=-100)
        elev_c = tomogram[4]
        elev_c = np.nan_to_num(elev_c, nan=1e6)

        self.trav = trav
        self.elev_g = elev_g

        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        self.planner.init_map(
            self.traversable_cost_threshold, 15, self.resolution, self.n_slice, 0.2,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
        )

    def plan(self, start_pos, end_pos, start_layer=0, end_layer=0,
             reference_height=0.0, optimize_path=True):
        if self.trav is None:
            raise RuntimeError('Tomogram has not been loaded')
        self.start_idx[0] = int(start_layer)
        self.end_idx[0] = int(end_layer)
        self.start_idx[1:] = self.pos2idx(start_pos)
        self.end_idx[1:] = self.pos2idx(end_pos)

        if not self._valid_index(self.start_idx) or not self._valid_index(self.end_idx):
            raise ValueError('Start or goal is outside tomogram bounds')

        # Keep the ROS-level optimize_path switch aligned with the native
        # planner.  Passing True unconditionally entered GPMP even for raw
        # A* requests; a same-cell start/goal then violated GPMP's minimum
        # two-point path precondition and aborted the whole PCT process.
        if not self.planner.plan(self.start_idx, self.end_idx, bool(optimize_path)):
            return None
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            return None

        if not optimize_path:
            # A* returns [layer, row, column] grid indices. Preserve the
            # discrete path and only convert it to map-frame XYZ coordinates.
            path = np.asarray(path, dtype=np.float64)
            if path.ndim != 2 or path.shape[1] < 3:
                raise ValueError('invalid A* result matrix')
            layers = np.rint(path[:, 0]).astype(np.int32)
            rows = np.rint(path[:, 1]).astype(np.int32)
            cols = np.rint(path[:, 2]).astype(np.int32)
            if (np.any(layers < 0) or np.any(layers >= self.n_slice) or
                    np.any(rows < 0) or np.any(rows >= self.map_dim[0]) or
                    np.any(cols < 0) or np.any(cols >= self.map_dim[1])):
                raise ValueError('A* result contains invalid grid indices')
            heights = self.elev_g[layers, rows, cols] / self.resolution
            raw_grid = np.stack([cols, rows, heights], axis=1)
            return transTrajGrid2Map(
                self.map_dim, self.center, self.resolution, raw_grid,
                reference_height=reference_height)

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()

        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(
            self.map_dim, self.center, self.resolution, traj_3d,
            reference_height=reference_height)

        return traj_3d

    def select_layer(self, pos, desired_ground_height, max_height_error):
        """Select the traversable surface at XY closest to a requested ground height."""
        return terrain_select_layer(
            self.trav, self.elev_g, pos, self.center, self.resolution,
            desired_ground_height, max_height_error, self.traversable_cost_threshold)

    def snap_to_traversable(self, pos, reference_height=0.0, radius_cells=3):
        """Project a base pose onto the nearest traversable tomogram cell.

        The tomogram stores ground height, whereas public ROS poses represent
        the robot base.  The returned Z therefore includes
        ``reference_height`` (normally the body height).
        """
        if self.trav is None or self.elev_g is None or self.elev_g_valid is None:
            raise RuntimeError('Tomogram has not been loaded')

        pos = np.asarray(pos, dtype=np.float64)
        if pos.shape[0] < 3 or not np.all(np.isfinite(pos[:3])):
            raise ValueError('Pose contains non-finite values')

        center_idx = self.pos2idx(pos[:2]).astype(np.int32)
        center_row, center_col = int(center_idx[1]), int(center_idx[0])
        if (center_row < 0 or center_col < 0 or
                center_row >= self.map_dim[0] or center_col >= self.map_dim[1]):
            raise ValueError('Pose is outside tomogram bounds')

        radius_cells = max(0, int(radius_cells))
        row_min = max(0, center_row - radius_cells)
        row_max = min(self.map_dim[0] - 1, center_row + radius_cells)
        col_min = max(0, center_col - radius_cells)
        col_max = min(self.map_dim[1] - 1, center_col + radius_cells)

        best = None
        for layer in range(self.n_slice):
            valid = self.elev_g_valid[layer, row_min:row_max + 1, col_min:col_max + 1]
            traversable = self.trav[layer, row_min:row_max + 1, col_min:col_max + 1] <= self.traversable_cost_threshold
            for local_row, local_col in np.argwhere(valid & traversable):
                row = row_min + int(local_row)
                col = col_min + int(local_col)
                candidate = np.array([
                    (row - self.offset[0]) * self.resolution + self.center[0],
                    (col - self.offset[1]) * self.resolution + self.center[1],
                    float(self.elev_g[layer, row, col]) + reference_height,
                ])
                distance = float(np.linalg.norm(candidate - pos[:3]))
                if best is None or distance < best[0]:
                    best = (distance, candidate, layer)

        if best is None:
            return None
        return best[1], best[2], best[0]

    def _valid_index(self, index):
        return (0 <= int(index[0]) < self.n_slice and
                0 <= int(index[1]) < self.map_dim[1] and
                0 <= int(index[2]) < self.map_dim[0])
    
    def pos2idx(self, pos):
        return terrain_pos2idx(pos, self.center, self.resolution, self.map_dim)
