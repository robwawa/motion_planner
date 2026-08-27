# SCAN-Planner MPC 生成器

本目录保存 `mpc_controller` 使用的 acados 代码生成结果。控制器源码位于
`../src/mpc_controller.cpp`，模型和代价函数由
`../scripts/generate_mpc_solver.py` 生成。

## 控制模型

MPC 使用平面三自由度状态和机体坐标系速度输入：

```text
x = [X, Y, yaw]
u = [vx_body, vy_body, wz]

dX/dt   = cos(yaw) * vx_body - sin(yaw) * vy_body
dY/dt   = sin(yaw) * vx_body + cos(yaw) * vy_body
dyaw/dt = wz
```

当前生成配置固定为 `N=12`、`dt=0.08 s`，预测时域为 `0.96 s`。Z 方向不在
该 MPC 状态中处理，机身高度和三维避障仍由规划器及其原有控制链路负责。

## 代价与约束

每个阶段的跟踪量为
`[X, Y, yaw, vx_body, vy_body, wz, dvx, dvy, dwz]`，其中后三项是相对上一
次控制输入的变化量。参考速度来自 B-spline 轨迹的前向差分；初始控制参数
`p=[vx_prev, vy_prev, wz_prev]` 用于抑制控制跳变。

生成时的默认权重为：

```text
Q = diag(8, 8, 1.5, 0.08, 0.08, 0.08, 0.4, 0.4, 0.4)
Qe = diag(12, 12, 2)
```

控制边界为 `vx_body ∈ [-0.8, 0.8] m/s`、`vy_body ∈ [-0.35, 0.35] m/s`、
`wz ∈ [-1.0, 1.0] rad/s`。运行时可以通过 ROS 参数调整权重和边界，但不能
改变生成器的状态/输入维度、预测步数或采样时间；这些修改需要重新生成代码。

## 生成与编译

首次配置或 acados 版本变化时，在工作空间根目录执行：

```bash
source /opt/ros/noetic/setup.bash
src/scan_planner/planner/plan_manage/scripts/setup_acados.sh
```

脚本会安装/构建 acados `v0.5.4`、HPIPM、BLASFEO 和 Python 绑定，并调用
`generate_mpc_solver.py` 生成本目录中的 C 文件。也可以在已有 acados 安装上
直接运行：

```bash
ACADOS_SOURCE_DIR=/path/to/acados \
python3 src/scan_planner/planner/plan_manage/scripts/generate_mpc_solver.py
```

生成后重新编译控制器：

```bash
catkin_make --pkg scan_planner -j4
```

不要手工修改生成的 `.c`/`.h` 文件；应修改 Python 生成脚本后重新生成。生成文件
会覆盖同名文件，且需要保持 acados 版本一致以避免 ABI 或求解器接口不匹配。

## 运行

默认控制器仍为 `closed_loop`。使用 MPC 时：

```bash
source devel/setup.bash
roslaunch scan_planner run.launch controller_mode:=mpc
```

控制器订阅 `/planning/bspline` 和机身位姿话题，发布 `/cmd_vel`，并在轨迹结束
或轨迹/里程计无效时输出零速度。航向误差过大时会暂时冻结平移、优先原地转向。

求解状态 `0` 表示成功；状态 `2` 表示达到最大迭代次数，控制器会记录警告并使用
当前可用迭代解。`mpc_controller/solver_timeout` 默认为 `0.008 s`，取当前
`0.01 s` 控制周期的 80%，其有效范围为 `[0.001, 0.020] s`。acados 返回
状态 `7` 或墙钟耗时超过该限制时，控制器会回滚本周期的轨迹时间、输出零速度、
发布 frozen、重置求解器，并在下一周期自动重试。该超时在求解迭代边界生效，
不是操作系统级的强制中断，也不用于检测机器人打滑或机械卡住。

其他非零状态和非有限输入/输出也会清空控制并停车。可通过
`mpc_controller/time_forward`、`mpc_controller/heading_error_threshold`、
`mpc_controller/max_vx`、`mpc_controller/max_vy`、`mpc_controller/max_vyaw`、
`mpc_controller/q_pos`、`mpc_controller/q_yaw`、`mpc_controller/r_velocity`、
`mpc_controller/r_rate`、`mpc_controller/solver_iterations` 和
`mpc_controller/solver_timeout` 调整运行行为。

更完整的控制器切换和 ROS 启动示例见
[`src/scan_planner/README.md`](../../../README.md) 及工作区
[`README.md`](../../../../../README.md)。
