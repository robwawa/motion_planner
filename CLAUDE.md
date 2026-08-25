# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概览

这是一个 ROS 1 Noetic 的 Catkin 工作空间，围绕四足机器人/通用底盘机器人在三维、多层环境中的全局/局部规划，包含两个主要规划模块和一个集成包：

- `src/pct_planner/`：PCT（Point Cloud Tomography）全局规划器。将 PCD 点云转换成多层地形 tomogram，并通过 `/pct/plan_path` Action 在多层可通行表面上做全局规划。
- `src/scan_planner/`：SCAN-Planner 局部空间碰撞感知规划器及仿真器。`planner/plan_manage` 是主包 `scan_planner`，其余 `plan_env`、`path_searching`、`bspline_opt`、`traj_utils` 是规划库，`simulator/` 提供点云地图、传感器和机器人运动学仿真。
- `src/pct_scan_navigation/`：PCT 到 SCAN 的 Python ROS 集成层，负责目标转换、Action 调用、全局路径校验/抽稀，以及向 SCAN 发布参考路径。

工作空间当前使用 `build/`、`devel/` 作为 Catkin 构建输出；这些目录是生成物，不应作为源码修改入口。

## 环境与依赖

目标环境是 Ubuntu 20.04 + ROS Noetic，基础 shell 操作前先加载 ROS 环境：

```bash
source /opt/ros/noetic/setup.bash
cd /home/wa/inspection/3D_motion_planner/motion_planner
```

主要依赖：

- ROS：`catkin`、`roscpp`/`rospy`、`actionlib`、消息包、TF、RViz 等；
- SCAN CPU 仿真/规划：Eigen、PCL、Armadillo，以及 ROS Noetic 相关包；
- PCT Python：Python 3、NumPy、SciPy、Open3D；
- PCT 原生规划库：GTSAM、OSQP、Eigen 和 pybind11，位于 `src/pct_planner/planner/lib/3rdparty/`，由专用脚本构建；
- CUDA/CuPy 和 OpenGL/GLEW/GLFW 是可选依赖。CPU 层析与 CPU 传感器仿真是默认、无需 GPU 的路径。

## 常用构建命令

### 构建整个 Catkin 工作空间

```bash
source /opt/ros/noetic/setup.bash
cd /home/wa/inspection/3D_motion_planner/motion_planner
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

开发时可用 `catkin_make -DCATKIN_ENABLE_TESTING=ON` 明确开启测试。只重建单个 Catkin 包时使用：

```bash
catkin_make --pkg scan_planner
catkin_make --pkg pct_planner
catkin_make --pkg pct_scan_navigation
```

`scan_planner` 主包依赖同一工作空间中的 `plan_env`、`path_searching`、`bspline_opt` 和 `traj_utils`；修改这些库后通常直接重新运行完整 `catkin_make` 最稳妥。

### 构建 PCT 原生库

PCT 的 `planner/lib` 不是普通 Catkin target，而是由脚本编译并由 Python wrapper 动态加载：

```bash
cd src/pct_planner/planner
./build_thirdparty.sh
./build.sh
```

`build_thirdparty.sh` 会重建 GTSAM 和 OSQP 的 `build/`/`install/`；`build.sh` 会构建 `planner/lib/src/` 下的 pybind11 模块并复制 `.so` 到 `planner/lib/`。若需要干净重建 PCT 主库，先删除 `src/pct_planner/planner/lib/build/`，再运行 `./build.sh`。构建后 ROS launch 通常会设置所需的 `LD_LIBRARY_PATH`；直接运行 Python 规划脚本时需要额外把 GTSAM 安装库加入环境，例如：

```bash
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$(pwd)/lib/3rdparty/gtsam-4.1.1/install/lib"
```

不要把 `planner/lib/3rdparty/` 的 vendored 源码或构建目录当作项目业务代码修改；PCT 顶层 CMake 只安装运行时原生库和 Python 脚本。

### 可选 GPU 后端

SCAN 的 GPU 传感器后端不是默认构建的一部分。安装 OpenGL 依赖后可重新配置：

```bash
sudo apt-get install libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
catkin_make -DUSE_GPU=ON
```

只有在 GPU 后端实际构建成功后才将 launch 参数设为 `use_gpu:=true`；否则使用 `use_gpu:=false` 的 CPU `pcl_render_node`。

## 测试与检查

项目没有统一的 lint/format target，也没有 `package.json` 或 Python packaging 流程。修改后至少运行与改动相关的构建和测试；不要把“没有 lint target”误认为测试已通过。

### PCT Python 层析回归测试

```bash
cd src/pct_planner/tomography/scripts
python3 -m unittest -v test_tomogram_backends.py
```

运行单个测试：

```bash
python3 -m unittest -v \
  test_tomogram_backends.CpuTomogramTest.test_cpu_output_contract
```

CUDA parity 测试在没有 CuPy 或 CUDA 设备时会自动跳过。测试从 `tomogram_cpu.py` 验证 CPU 输出契约、NaN/半值取整和多线程膨胀结果；若改动 CUDA/CPU 层析逻辑，应同时关注两种后端的数组布局和 NaN 语义。

### Catkin/C++ 测试

完整 Catkin 测试（需要先构建）：

```bash
cd /home/wa/inspection/3D_motion_planner/motion_planner
source devel/setup.bash
catkin_make run_tests
catkin_test_results
```

直接运行 SCAN 的单个 GTest 或筛选单个 case：

```bash
./devel/lib/scan_planner/reference_path_z_test
./devel/lib/scan_planner/reference_path_z_test \
  --gtest_filter=ReferencePathZProfile.SamplesLinearSlope
```

PCT 原生库测试在其独立 CMake build 目录中：

```bash
cd src/pct_planner/planner/lib/build
ctest --output-on-failure
ctest --output-on-failure -R a_star_reset_regression
```

已有测试重点包括 `ReferencePathZProfile` 的路径高度投影/采样和 A* 搜索状态重置；新增或修改路径高度、A* 状态或层析后端时优先运行这些回归测试。

## 运行与调试入口

### 独立 SCAN CPU 仿真

终端 1：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roscore
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch scan_planner run.launch \
  is_real_world:=false \
  navi_mode:=1 \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  use_gpu:=false
```

RViz 可另启：

```bash
roslaunch scan_planner rviz.launch
```

`run.launch` 通过 `advanced_param.xml` 装载 SCAN 主节点参数，通过 `simulator.xml` 选择 mockamap 或 PCD 地图、CPU/GPU 传感器和初始位姿。`navi_mode` 含义为：`1` RViz 目标，`2` 预设航点，`3` 跟踪参考路径并进行局部避障。`controller_mode:=open_loop` 用于观察三维轨迹播放，`closed_loop` 用于 `/cmd_vel` 跟踪；现有运动学仿真器不会依据地形主动更新 Z，不要把它当作楼梯动力学验证。

### PCT 单独运行

通常先生成 tomogram，再启动规划器。PCT launch 的 `tomogram_name` 必须与 `rsc/tomogram/<name>.pickle` 对应，且应与输入 PCD 匹配：

```bash
roslaunch pct_planner tomography.launch \
  backend:=cpu \
  pcd_map_file:=$(rospack find pct_planner)/rsc/pcd/building2_9.pcd \
  tomogram_name:=building2_9
roslaunch pct_planner planner.launch tomogram_name:=building2_9
```

也可一键启动层析、全局规划和 RViz：

```bash
roslaunch pct_planner pct_planner.launch backend:=cpu launch_rviz:=true
```

层析节点读取 PCD，运行 CPU/CUDA 后端，把 tomogram 保存到 `src/pct_planner/rsc/tomogram/`，并发布 `/global_points`、`/tomogram`、`/pct/terrain_map` 等话题。规划节点等待指定 pickle 后，通过原生 `a_star`/轨迹优化模块提供 `/pct/plan_path`，并 latch 发布 `/pct/global_path`。

### PCT + SCAN 分层导航

```bash
roslaunch pct_scan_navigation pct_scan_demo.launch controller_mode:=open_loop
```

另一个终端发布三维目标：

```bash
rostopic pub -1 /goal_pose_3d geometry_msgs/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: -6.0, y: -1.0, z: 4.9}, orientation: {w: 1.0}}}'
```

集成 launch 让 PCT 和 SCAN 共用同一 PCD 与 `map` 坐标系：`navigation_manager.py` 订阅目标和机身里程计，调用 `/pct/plan_path`，验证/抽稀路径后发布 `/navigation/reference_path`；SCAN 的 `navi_mode=3` 接收该路径并做局部 B-spline 避障。`goal_interactive_marker.py` 提供 RViz 交互式三维目标和 Plan 菜单。

## 架构与关键数据流

### PCT 全局规划链路

1. `pct_tomography_node.py` 是 ROS 可执行包装器，加载 `tomography/scripts/tomography.py`。`Tomography` 从 PCD 建立网格，选择 `CpuTomogram` 或 CUDA `Tomogram`，生成 traversability、地面高度和梯度，并导出 pickle。
2. `pct_planner_node.py` 是 `/pct/plan_path` 的 Action server。它动态加入 `planner/scripts` 和 native library 路径，创建 `TomogramPlanner`，等待 tomogram，校验 frame/有限值，投影起终点到可通行层，再执行 A*；`optimize_path` 决定使用离散 A* 路径还是原生轨迹优化。
3. `pct_planner/action/PlanPath3D.action` 定义成功、坐标系不匹配、越界、无通行层、无路径和抢占等状态，同时返回 snapped endpoints；`PctTerrainMap.msg` 是给局部规划器消费的完整地形缓存。

PCT 数组契约很重要：C-order 布局为 `[layer][row][col]`，`row` 对应 PCT X 网格索引、`col` 对应 Y 网格索引，不能把用于 RViz 的 `PointCloud2` 视觉层当作完整地形缓存。路径和地形消息默认使用 `map` frame。

### PCT/SCAN 集成链路

`pct_scan_navigation` 不负责重新规划算法，只负责接口编排：

- 将 RViz 2D goal 提升为带当前机身 Z 的 3D goal，并用 TF 转到 `navigation_frame`；
- 用 odometry 作为起点调用 PCT Action；
- 拒绝 frame 错误、无效路径、路径离机身过远等结果；
- 把路径裁剪到机器人最近点并发布给 SCAN；
- 将 PCT snapped goal 发布到 `/navigation/validated_goal`，将状态发布到 `/navigation/status`。

### SCAN 局部规划链路

- `plan_env/GridMap` 融合深度/激光点云、射线和膨胀障碍物；`getPlanningOccupancy()` 是 A*、B-spline rebound 和 FSM 共用的碰撞入口。
- `plan_env/PctTerrainMap` 将 `/pct/terrain_map` 校验并缓存。通过 `terrain_check/use_pct_traversability` 启用后，PCT traversability/高度约束与 SCAN 膨胀障碍共同决定规划占用；PCT 地图不可用时当前实现会回退到 SCAN 膨胀占用。
- `scan_replan_fsm.cpp` 管理 INIT/WAIT_TARGET/GEN_NEW_TRAJ/REPLAN_TRAJ/EXEC_TRAJ/EMERGENCY_STOP 状态，按 `navi_mode` 接收目标、航点或路径；`planner_manager.cpp` 生成全局多项式/最小 snap 参考，并将局部初始路径交给 `BsplineOptimizer`。
- `path_searching/dyn_a_star.cpp` 在局部障碍段上做三维 A* rebound；`bspline_opt` 做平滑、碰撞和动力学约束优化；`traj_utils` 提供多项式轨迹和可视化。
- 参考路径模式 `min_snap_single_pass` 与 `polyline_rolling_window` 都可使用路径高度初始化。`reference_path_z_mode:=base` 表示输入 Z 已是机身基座高度；`ground` 表示输入 Z 是地面高度，SCAN 会加一次 `grid_map/body_height`。该高度 profile 只用于局部 B-spline 初值，碰撞优化仍可调整 Z。

## 参数、坐标系与修改注意事项

- PCT/SCAN/集成 demo 默认都使用 `map` frame；修改 frame 时要同时检查 PCD 发布、odom、TF、RViz、`PctTerrainMap.header.frame_id` 和 `grid_map/frame_id`。
- PCT 公共 profile 来自 `src/pct_planner/config/pct_profile.yaml`，由 `pct_profile.py` 校验后供层析和规划器使用。不要只修改一侧的私有参数，否则可能导致 tomogram 与局部 traversability 阈值不一致。
- 集成 demo 的 `pcd_map_file` 与 `tomogram_name` 必须成对覆盖；一个新的 PCD 应生成唯一的 tomogram 名称，避免规划器静默加载另一张地图的 pickle。
- `reference_path_z_mode`、`reference_path_mode` 和 `navi_mode` 是行为开关，非法值会在 SCAN FSM 初始化时关闭节点。修改路径预处理时要保留至少两个有效点、有限坐标以及起点到当前机器人距离检查。
- PCT endpoint Z 代表机身 base pose；PCT tomogram 内部存储的是 ground elevation。新增接口时要明确是否需要加 `body_height`，避免重复加高度。
- 修改 ROS 消息、Action 或参数名后，需要同步更新对应 `package.xml`/`CMakeLists.txt`、launch、Python import、C++ subscriber/publisher 及所有相关测试。
- PCT 的 `rsc/pcd`、tomogram 等地图/运行时资源可能是本地大文件，并受 `.gitignore` 规则影响；优先使用现有资源或明确生成路径，不要把生成的 build/devel、pickle 和缓存误加入提交。

## 主要入口文件

- 全局规划：`src/pct_planner/scripts/pct_planner_node.py`、`src/pct_planner/tomography/scripts/tomography.py`、`src/pct_planner/planner/scripts/planner_wrapper.py`
- 集成：`src/pct_scan_navigation/scripts/navigation_manager.py`
- SCAN FSM/局部规划：`src/scan_planner/planner/plan_manage/src/scan_replan_fsm.cpp`、`planner_manager.cpp`
- 地图占用与 PCT 约束：`src/scan_planner/planner/plan_env/src/grid_map.cpp`、`pct_terrain_map.cpp`
- 默认参数/启动：`src/scan_planner/planner/plan_manage/launch/run.launch`、`advanced_param.xml`、`simulator.xml`，以及 `src/pct_scan_navigation/launch/pct_scan_demo.launch`
