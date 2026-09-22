# global_pct_planner

CPU/OpenMP C++ implementation of the PCT tomography and global planner.  It
publishes the existing `/pct/*` topics, but its generated ROS interfaces live
in the `global_pct_planner` namespace.

The normal entry point is:

```bash
roslaunch global_pct_planner global_pct_planner.launch \
  pcd_map_file:=/path/to/cloud.pcd
```

Tomograms use the versioned binary `.pctm` format.  Legacy Python pickle maps
can be converted once with:

```bash
rosrun global_pct_planner convert_pickle_to_pctm.py old.pkl new.pctm
```

对同一份旧地图执行数值、NaN 布局和通行性分类回归：

```bash
rosrun global_pct_planner compare_legacy_pct.py old.pkl new.pctm
```

旧版 pickle 若由导出器以 float16 保存，工具会单独报告量化误差，并仍
要求通行性分类完全一致；float32 CPU 参考则按 1e-4 误差验收。

The launch files explicitly derive the source package path and pass these
locations to the C++ nodes:

```text
<source_package_path>/rsc/pcd
<source_package_path>/rsc/tomogram
```

The C++ nodes only consume `pcd_file`, `export_dir`, and `tomogram_dir`; they do
not resolve or fall back to package paths.  PCD and `.pctm` resources are
intentionally not installed into `share`; they remain in the source package.
Existing copies under `install/share/global_pct_planner/rsc` are stale install
artifacts and must be removed by cleaning the install space.

The GPMP/A*/map/OSQP adapter sources are copied under `native/src` and are
compiled directly by this package; no old package native target or pybind
module is built.  GTSAM and OSQP remain configurable local C++ dependencies
through `PCT_GTSAM_DIR`, `PCT_METIS_LIB_DIR`, and `PCT_OSQP_DIR`.  The existing
`pct_planner` package is intentionally left independent and its source files
are not modified.
