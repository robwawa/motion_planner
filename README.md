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

## 目录布局

SCAN-Planner 源码位于 `src/scan_planner/`，其中 `planner/plan_manage` 提供主包 `scan_planner`，`planner/plan_env`、`path_searching`、`bspline_opt` 和 `traj_utils` 为规划依赖，`simulator/` 包含仿真和可视化组件。
