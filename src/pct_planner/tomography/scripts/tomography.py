#!/usr/bin/python3
import os
import sys
import time
import pickle
import numpy as np
import open3d as o3d
  
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from pct_planner.msg import PctTerrainMap

tomography_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, tomography_root)
sys.path.insert(0, os.path.join(tomography_root, 'config'))
from config import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from config import Config
from pct_profile import load_public_profile

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


def create_tomogram(profile, backend):
    """Create the requested backend without requiring CuPy for CPU mode."""
    if backend == 'cpu':
        from tomogram_cpu import CpuTomogram
        return CpuTomogram(profile), 'cpu'
    try:
        import cupy as cp
        cp.cuda.runtime.getDeviceCount()
        from tomogram import Tomogram
        return Tomogram(profile), 'cuda'
    except Exception as error:
        if backend == 'cuda':
            raise RuntimeError('CUDA backend unavailable: {}'.format(error))
        rospy.logwarn('CUDA unavailable; using CPU backend: %s', error)
        from tomogram_cpu import CpuTomogram
        return CpuTomogram(profile), 'cpu'


class Tomography(object):
    def __init__(self, cfg, profile, backend='auto', pcd_file=None,
                 tomogram_name=None):
        self.cfg = cfg
        export_dir = rospy.get_param('~export_dir', cfg.map.export_dir)
        self.export_dir = export_dir if os.path.isabs(export_dir) else os.path.join(
            rsg_root, export_dir)
        self.pcd_file = pcd_file
        self.tomogram_name = tomogram_name
        self.resolution = profile.map.resolution
        self.ground_h = profile.map.ground_h
        self.slice_dh = profile.map.slice_dh

        self.center = np.zeros(2, dtype=np.float32)
        cache_path = os.path.join(self.export_dir, self.tomogram_name + '.pickle')
        if os.path.isfile(cache_path):
            self.loadCachedTomogram(cache_path)
            return

        self.tomogram, self.backend = create_tomogram(profile, backend)
        rospy.loginfo('Tomogram backend: %s', self.backend)
        points = self.loadPCD(self.pcd_file)

        # Process
        self.process(points)

    def loadCachedTomogram(self, cache_path):
        with open(cache_path, 'rb') as handle:
            data_dict = pickle.load(handle)

        tomogram = np.asarray(data_dict['data'], dtype=np.float32)
        if tomogram.ndim != 4 or tomogram.shape[0] != 5:
            raise RuntimeError('invalid cached tomogram: {}'.format(cache_path))

        self.resolution = float(data_dict['resolution'])
        self.center = np.asarray(data_dict['center'], dtype=np.float32)
        self.slice_h0 = float(data_dict['slice_h0'])
        self.slice_dh = float(data_dict['slice_dh'])
        self.n_slice = tomogram.shape[1]
        self.map_dim_x = tomogram.shape[2]
        self.map_dim_y = tomogram.shape[3]
        self.VISPROTO_I, self.VISPROTO_P = \
            GRID_POINTS_XYZI(self.resolution, self.map_dim_x, self.map_dim_y)

        layers_t, _, _, layers_g, layers_c = tomogram
        rospy.loginfo('Reusing cached tomogram: %s', cache_path)
        self.initROS()
        self.publishLayers(self.layer_G_pub_list, layers_g, layers_t)
        self.publishLayers(self.layer_C_pub_list, layers_c, None)
        self.publishTomogram(layers_g, layers_t)
        self.publishTerrainMap(layers_g, layers_t)

    def initROS(self):
        self.map_frame = rospy.get_param('~map_frame', self.cfg.ros.map_frame)

        pointcloud_topic = rospy.get_param('~pointcloud_topic', self.cfg.ros.pointcloud_topic)
        self.pointcloud_pub = rospy.Publisher(pointcloud_topic, PointCloud2, latch=True, queue_size=1)

        self.layer_G_pub_list = []
        self.layer_C_pub_list = []
        layer_G_topic = rospy.get_param('~layer_G_topic', self.cfg.ros.layer_G_topic)
        layer_C_topic = rospy.get_param('~layer_C_topic', self.cfg.ros.layer_C_topic)
        for i in range(self.n_slice):
            layer_G_pub = rospy.Publisher(layer_G_topic + str(i), PointCloud2, latch=True, queue_size=1)
            self.layer_G_pub_list.append(layer_G_pub)
            layer_C_pub = rospy.Publisher(layer_C_topic + str(i), PointCloud2, latch=True, queue_size=1)
            self.layer_C_pub_list.append(layer_C_pub)

        tomogram_topic = rospy.get_param('~tomogram_topic', self.cfg.ros.tomogram_topic)
        self.tomogram_pub = rospy.Publisher(tomogram_topic, PointCloud2, latch=True, queue_size=1)
        terrain_map_topic = rospy.get_param('~terrain_map_topic', self.cfg.ros.terrain_map_topic)
        self.terrain_map_pub = rospy.Publisher(terrain_map_topic, PctTerrainMap, latch=True, queue_size=1)

    def loadPCD(self, pcd_file):
        path = pcd_file if os.path.isabs(pcd_file) else os.path.join(
            rsg_root, "rsc", "pcd", pcd_file)
        pcd = o3d.io.read_point_cloud(path)
        points = np.asarray(pcd.points).astype(np.float32)
        if points.size == 0:
            raise RuntimeError("cannot load point cloud: {}".format(path))
        rospy.loginfo("PCD points: %d", points.shape[0])
        if points.shape[1] > 3:
            points = points[:, :3]
        self.points_max = np.max(points, axis=0)
        self.points_min = np.min(points, axis=0)           
        self.points_min[-1] = self.ground_h
        self.map_dim_x = int(np.ceil((self.points_max[0] - self.points_min[0]) / self.resolution)) + 4
        self.map_dim_y = int(np.ceil((self.points_max[1] - self.points_min[1]) / self.resolution)) + 4
        n_slice_init = int(np.ceil((self.points_max[2] - self.points_min[2]) / self.slice_dh))
        self.center = (self.points_max[:2] + self.points_min[:2]) / 2
        self.slice_h0 = self.points_min[-1] + self.slice_dh
        self.tomogram.initMappingEnv(self.center, self.map_dim_x, self.map_dim_y, n_slice_init, self.slice_h0)

        rospy.loginfo("Map center: [%.2f, %.2f]", self.center[0], self.center[1])
        rospy.loginfo("Dim_x: %d", self.map_dim_x)
        rospy.loginfo("Dim_y: %d", self.map_dim_y)
        rospy.loginfo("Num slices init: %d", n_slice_init)

        self.VISPROTO_I, self.VISPROTO_P = \
            GRID_POINTS_XYZI(self.resolution, self.map_dim_x, self.map_dim_y)

        return points
        
    def process(self, points):        
        t_map = 0.0
        t_trav = 0.0
        t_simp = 0.0
        t_all = 0.0
        n_repeat = 10 if self.backend == 'cuda' else 1
        n_warmup = 1 if self.backend == 'cuda' else 0

        """ 
        CUDA uses one warm-up run followed by repeated timing.  CPU runs once so
        that generating a large tomogram on a CUDA-free workstation remains
        practical.
        """
        for i in range(n_repeat + n_warmup):
            t_start = time.time()
            layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, timings = self.tomogram.point2map(points)

            if i >= n_warmup:
                t_map += timings['t_map']
                t_trav += timings['t_trav']
                t_simp += timings['t_simp']
                t_all += (time.time() - t_start) * 1e3

        rospy.loginfo("Num slices simp: %d", layers_g.shape[0])
        rospy.loginfo("Num repeats (for benchmarking only): %d", n_repeat)
        rospy.loginfo(" -- avg t_map  (ms): %f", t_map / n_repeat)
        rospy.loginfo(" -- avg t_trav (ms): %f", t_trav / n_repeat)
        rospy.loginfo(" -- avg t_simp (ms): %f", t_simp / n_repeat)
        rospy.loginfo(" -- avg t_all  (ms): %f", t_all / n_repeat)

        self.n_slice = layers_g.shape[0]

        self.exportTomogram(
            np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c)),
            self.tomogram_name)

        self.initROS()
        self.publishPoints(points)
        self.publishLayers(self.layer_G_pub_list, layers_g, layers_t)
        self.publishLayers(self.layer_C_pub_list, layers_c, None)
        self.publishTomogram(layers_g, layers_t)
        self.publishTerrainMap(layers_g, layers_t)

    def exportTomogram(self, tomogram, map_file):        
        data_dict = {
            'data': tomogram.astype(np.float16),
            'resolution': self.resolution,
            'center': self.center,
            'slice_h0': self.slice_h0,
            'slice_dh': self.slice_dh,
        }
        file_name = map_file + '.pickle'
        with open(self.export_dir + file_name, 'wb') as handle:
            pickle.dump(data_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

        rospy.loginfo("Tomogram exported: %s", file_name)

    def publishPoints(self, points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.map_frame

        point_msg = pc2.create_cloud_xyz32(header, points)
        self.pointcloud_pub.publish(point_msg)

    def publishLayers(self, pub_list, layers, color=None):
        header = Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = self.map_frame

        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.center

        for i in range(layers.shape[0]):
            layer_points[:, 2] = layers[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            if color is not None:
                layer_points[:, 3] = color[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            else:
                layer_points[:, 3] = 1.0
        
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, valid_points)
            pub_list[i].publish(points_msg) 

    def publishTomogram(self, layers_g, layers_t):
        header = Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = self.map_frame

        n_slice = layers_g.shape[0]
        vis_g = layers_g.copy()
        vis_t = layers_t.copy() 
        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.center

        global_points = None
        for i in range(n_slice - 1):
            mask_h = (vis_g[i + 1] - vis_g[i]) < self.slice_dh
            vis_g[i, mask_h] = np.nan
            vis_t[i + 1, mask_h] = np.minimum(vis_t[i, mask_h], vis_t[i + 1, mask_h])
            layer_points[:, 2] = vis_g[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            layer_points[:, 3] = vis_t[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            if global_points is None:
                global_points = valid_points
            else:
                global_points = np.concatenate((global_points, valid_points), axis=0)

        layer_points[:, 2] = vis_g[-1, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
        layer_points[:, 3] = vis_t[-1, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
        valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
        global_points = np.concatenate((global_points, valid_points), axis=0)
        
        points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, global_points)
        self.tomogram_pub.publish(points_msg)

    def publishTerrainMap(self, layers_g, layers_t):
        """Publish the complete, non-visual PCT cache once for local planners.

        Arrays deliberately preserve NumPy C-order [layer, row, col].  The
        consumer must not infer the layout from the visual PointCloud2 topic.
        """
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.map_frame
        msg = PctTerrainMap()
        msg.header = header
        msg.resolution = float(self.resolution)
        msg.center_x = float(self.center[0])
        msg.center_y = float(self.center[1])
        msg.layers = int(layers_g.shape[0])
        msg.rows = int(layers_g.shape[1])
        msg.cols = int(layers_g.shape[2])
        msg.traversability = np.ascontiguousarray(layers_t, dtype=np.float32).ravel().tolist()
        msg.ground_elevation = np.ascontiguousarray(
            np.nan_to_num(layers_g, nan=0.0), dtype=np.float32).ravel().tolist()
        msg.elevation_valid = np.ascontiguousarray(
            np.isfinite(layers_g), dtype=np.uint8).ravel().tolist()
        self.terrain_map_pub.publish(msg)
        rospy.loginfo('PCT terrain map published: %d layers, %d x %d cells on %s',
                      msg.layers, msg.rows, msg.cols, self.terrain_map_pub.resolved_name)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=('auto', 'cuda', 'cpu'), default='auto', help='Tomogram backend (default: auto)')
    parser.add_argument('--pcd-file', required=True,
                        help='PCD path relative to rsc/pcd or an absolute path')
    parser.add_argument('--tomogram-name', required=True,
                        help='Output basename under rsc/tomogram, without .pickle')
    args = parser.parse_args()

    cfg = Config()

    rospy.init_node('pointcloud_tomography')
    profile = load_public_profile()
    rospy.loginfo('PCT traversability: kernel=%d slope=%.3f step=%.3f barrier=%.3f threshold=%.3f',
                  profile.trav.kernel_size, profile.trav.slope_max,
                  profile.trav.step_max, profile.trav.cost_barrier,
                  profile.trav.cost_threshold)
    mapping = Tomography(cfg, profile, args.backend, args.pcd_file,
                          args.tomogram_name)

    rospy.spin()
