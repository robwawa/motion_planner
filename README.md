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

RViz 可单独启动：

```bash
roslaunch scan_planner rviz.launch
```

默认仿真地图由 `mockamap` 生成，CPU 传感器节点发布点云，局部规划器输出规划轨迹。真实机器人模式需将 `is_real_world` 设为 `true`，并按机器人实际话题修改参数。

## PCT + SCAN 分层导航 Demo

首期集成使用同一份 Building PCD：PCT 通过 `/pct/plan_path` Action 生成全局多层路径，
`navigation_manager` 将其校验、抽稀后发布到 SCAN 的
`/navigation/reference_path`。所有路径均以 `map` 为坐标系，z 表示
`base_link` 高度。

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

## 目录布局

SCAN-Planner 源码位于 `src/scan_planner/`，其中 `planner/plan_manage` 提供主包 `scan_planner`，`planner/plan_env`、`path_searching`、`bspline_opt` 和 `traj_utils` 为规划依赖，`simulator/` 包含仿真和可视化组件。
