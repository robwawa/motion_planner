# motion_planner ROS 工作空间

该工作空间同时包含：

- `pct_planner`：PCT 全局规划与三维层析节点；
- `scan_planner`：SCAN-Planner 空间碰撞感知局部规划器及其仿真器。

## 编译

```bash
source /opt/ros/noetic/setup.bash
cd /home/wa/inspection/3D_motion_planner/motion_planner
catkin_make
source devel/setup.bash
```

PCT 的 CPU 原生扩展默认使用 ROS Noetic 的系统 Python 构建：

```bash
src/pct_planner/planner/build.sh
```

也可以通过 `PCT_PYTHON_EXECUTABLE=/path/to/python3` 显式选择 Python；生成扩展的
`cpython-XY` ABI 必须与运行 PCT 节点的 Python 一致。

SCAN-Planner 的 CPU 仿真需要 Armadillo、PCL、Eigen 和 ROS Noetic 的相关消息包。GPU 渲染不是基础构建的必要条件。

## 启动 SCAN-Planner 仿真

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roscore
```

另一个终端运行：

```bash
source /opt/ros/noetic/setup.bash
source /home/wa/inspection/3D_motion_planner/motion_planner/devel/setup.bash
roslaunch scan_planner run.launch \
  is_real_world:=false \
  navi_mode:=1 \
  sensor_type:=lidar \
  controller_mode:=closed_loop \
  use_gpu:=false
```

将 `controller_mode` 改为 `mpc` 可启用 acados MPC 控制器：

```bash
roslaunch scan_planner run.launch controller_mode:=mpc
```

MPC 的模型、权重、约束、acados 生成和故障处理说明见
[`mpc_generated/README.md`](src/scan_planner/planner/plan_manage/mpc_generated/README.md)。

RViz 可单独启动：

```bash
roslaunch scan_planner rviz.launch
```

## 启动 Gazebo 仿真

Gazebo 仿真使用与 `3d-navi` 相同的 Building 世界、Unitree A1 模型、12 关节控制器、
Gazebo 状态里程计和 Livox 点云链路。先使用无强化学习控制器的模式验证基础仿真：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch pct_scan_gazebo gazebo.launch \
  gui:=false headless:=true paused:=false launch_rl:=false
```

完整的 PCT + SCAN + Gazebo 仿真分两步启动。先启动 A1、传感器和强化学习控制器：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch pct_scan_gazebo gazebo.launch \
  gui:=true paused:=false launch_rl:=true
```

确认 12 个关节控制器加载完成后，在另一终端运行规划侧车：

```bash
export CONDA_ENV_PATH=/path/to/inspection
roslaunch pct_scan_gazebo pct_scan_navigation_sidecar.launch \
  controller_mode:=closed_loop
```

PCT CPU 节点使用 ROS Noetic 的系统 Python；`CONDA_ENV_PATH` 仅供强化学习控制器
加载 TorchScript 模型。基础 Gazebo 仿真不依赖 Conda；设置 `launch_rl:=false`
即可跳过 TorchScript 控制器。Gazebo A1 的导航机身高度默认使用其稳定站立高度
`0.315 m`；`init_z:=0.6` 仅用于机器人生成时避免与地面相交。

默认仿真地图由 `mockamap` 生成，CPU 传感器节点发布点云，局部规划器输出规划轨迹。真实机器人模式需将 `is_real_world` 设为 `true`，并按机器人实际话题修改参数。

## PCT + SCAN 分层导航 Demo

首期集成使用同一份 Building PCD：PCT 通过 `/pct/plan_path` Action 生成全局多层路径，
`navigation_manager` 将其校验、抽稀后发布到 SCAN 的
`/navigation/reference_path`。所有路径均以 `map` 为坐标系；路径 Z 的语义由
`reference_path_z_mode` 指定（默认 `base`）。

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch pct_scan_navigation pct_scan_demo.launch controller_mode:=open_loop
```

另一个终端发布 3D 目标（起点来自 `/quad_0/body_pose`）：

```bash
rostopic pub -1 /goal_pose_3d geometry_msgs/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: -6.0, y: -1.0, z: 4.9}, orientation: {w: 1.0}}}'
```

`open_loop` 用于验证多层 z 轨迹；`closed_loop` 适合验证同楼层的
`/cmd_vel` 跟踪与局部避障。现有运动学仿真器不会根据地形主动更新 z，
因此后者不应作为楼梯动力学验证。

### 模式 3 的参考路径高度处理

`navi_mode:=3` 是 PCT 全局路径到 SCAN 局部避障的跟踪模式。SCAN 保持自身的
XY 局部规划和三维碰撞优化，但会将局部 B 样条的 Z 初值按机器人在 PCT 路径上的
前进距离采样；因此楼梯、坡面等中间高度变化不会再被简单地按局部起终点线性连接。

- `reference_path_z_mode:=base`：输入路径的 Z 已是机身基座高度，直接使用；
- `reference_path_z_mode:=ground`：输入路径的 Z 是地面高度，SCAN 在接收时加一次
  `grid_map/body_height`（默认 `0.4 m`）。

上述高度参考只用于初始化，SCAN 的碰撞避障仍可按局部点云上、下调整轨迹。该优化
仅在模式 3 生效；模式 1（RViz 目标）和模式 2（预设航点）继续使用原有的起终点线性
Z 初始化。`reference_path_mode:=min_snap_single_pass` 与
`reference_path_mode:=polyline_rolling_window` 都支持该行为。

## 目录布局

SCAN-Planner 源码位于 `src/scan_planner/`，其中 `planner/plan_manage` 提供主包 `scan_planner`，`planner/plan_env`、`path_searching`、`bspline_opt` 和 `traj_utils` 为规划依赖，`simulator/` 包含仿真和可视化组件。
