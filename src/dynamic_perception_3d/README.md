# dynamic_perception_3d

`dynamic_perception_3d` 是一个 ROS Noetic 动态障碍感知节点。它将实时
LiDAR 点云与静态 PCD 地图进行逐点差分，对剩余点进行聚类，并维护稳定的
三维动态障碍物。

模块只输出世界坐标系下的动态障碍点云，不依赖、也不了解 PCT layer、
Tomogram、A* 或重规划状态机。

## 处理流程

```text
实时 LiDAR 点云
  ↓ TF 转换到 map
预处理（NaN、机体、ROI、FOV、VoxelGrid）
  ↓
静态 PCD KD-Tree 逐点差分
  ↓
动态候选点聚类与 cluster-level 静态复检
  ↓
障碍关联、确认、遮挡保持、可见性清除
  ↓
/dynamic_perception/dynamic_cloud
```

## 运行

```bash
source devel/setup.bash
roslaunch dynamic_perception_3d dynamic_perception.launch \
  static_pcd_file:=$(rospack find pct_planner)/rsc/pcd/building2_9.pcd
```

除 `static_pcd_file` 外，节点参数均从
`config/dynamic_perception.yaml` 读取。该唯一 launch 参数保留给上层规划器共享
`pcd_map_file`，确保 Tomogram、静态过滤和动态感知始终使用同一份 PCD。
场景差异通过 `scenario_config_file` 指定的 YAML 覆盖文件处理；覆盖文件在基础 YAML
之后加载，只需包含需要改写的参数。

Gazebo 主仿真会自动启动该模块：

```bash
roslaunch pct_scan_gazebo pct_scan_gazebo_demo.launch
```

主 Demo 默认使用 `pct_planner/rsc/pcd/building2_9.pcd` 作为感知静态地图，
并通过 `enable_dynamic_perception:=false` 关闭独立感知节点。该启动只发布
动态感知结果，不会修改现有规划器的动态输入、A* 或重规划触发逻辑。

主输出话题为：

```text
/dynamic_perception/dynamic_cloud
```

消息类型为 `sensor_msgs/PointCloud2`，坐标系为 `map`。开启
`publish_debug:=true` 后，还会发布当前 LiDAR、静态匹配点、静态表面匹配点、动态候选点、
确认障碍点、障碍 marker 和可见性射线等调试话题，均位于
`/dynamic_perception/` 命名空间下。

`/dynamic_perception/scan_healthy`（`std_msgs/Bool`）表示本帧是否具备动态
清除所需的有效扫描证据；为 `false` 时，节点保留并重新发布上一帧有效动态快照。

## 话题说明

除输入话题外，以下话题均使用 `map_frame`（默认 `map`）坐标系。调试话题只有在
`publish_debug:=true` 时才会创建。

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `lidar_topic`（默认 `/livox/Pointcloud2`） | `sensor_msgs/PointCloud2` | 输入实时 LiDAR 点云。节点先生成完整有效的传感器可见性扫描，再按 base 局部窗口与高度门控生成用于静态差分的标记扫描。 |
| `/dynamic_perception/dynamic_cloud` | `sensor_msgs/PointCloud2` | 主输出。由已确认（`CONFIRMED`）或暂时遮挡（`OCCLUDED`）的动态障碍体素组成，供其他模块订阅。 |
| `/dynamic_perception/scan_healthy` | `std_msgs/Bool` | 表示当前帧是否具有足够的有效扫描和 TF 证据。`false` 时不推进清除，并保留上一帧动态快照。 |
| `/dynamic_perception/current_lidar` | `sensor_msgs/PointCloud2` | 当前帧经过预处理、标记高度门控和体素下采样后的扫描点，尚未进行 PCD 静态剔除。 |
| `/dynamic_perception/static_map` | `sensor_msgs/PointCloud2` | 已加载并按 `static_map_pose` 转换到 `map` 的完整静态 PCD；使用 latched 发布，便于 RViz 显示。 |
| `/dynamic_perception/static_matched_cloud` | `sensor_msgs/PointCloud2` | 当前扫描中被判定为静态的点，包括 PCD 半径匹配、静态地面先验和法向表面匹配结果。它是当前帧结果，不是完整 PCD。 |
| `/dynamic_perception/static_surface_matched_cloud` | `sensor_msgs/PointCloud2` | `static_matched_cloud` 中通过 PCD 法向量局部表面测试命中的子集，用于诊断墙面/地面采样空隙和法向距离阈值。 |
| `/dynamic_perception/dynamic_candidate_cloud` | `sensor_msgs/PointCloud2` | 首次静态 PCD 差分后剩余的候选点，尚未完成聚类级静态复检和多帧确认；其中可能包含静态残留。 |
| `/dynamic_perception/dynamic_confirmed_cloud` | `sensor_msgs/PointCloud2` | 调试副本，内容与当前 `/dynamic_perception/dynamic_cloud` 相同，包含确认或遮挡保持中的动态障碍。 |
| `/dynamic_perception/dynamic_obstacle_markers` | `visualization_msgs/MarkerArray` | 在 RViz 中显示障碍物轨迹/包围盒及生命周期状态；颜色可区分暂定、确认和遮挡状态。 |
| `/dynamic_perception/clearing_rays` | `visualization_msgs/MarkerArray` | 显示当前帧用于旧体素可见性和清除判断的传感器到体素射线；红色为遮挡、蓝色为旧点邻域仍有回波、绿色为本帧已清除、灰色为未知。 |
| `/dynamic_perception/history_window` | `visualization_msgs/Marker` | 以当前 `base_frame` 位姿为中心显示动态障碍历史体素的保留窗口；青绿色线框，XY 半边长为 `perception_window_size`，Z 范围由 `history_window_min_height` / `history_window_max_height` 决定。 |
| `/dynamic_perception/preprocess_window` | `visualization_msgs/Marker` | 以当前 `sensor_frame` 位姿显示传感器 FOV 与最小/最大量程的黄色线框包络，用于诊断射线清除可使用的观测范围。 |

其中 `static_matched_cloud`、`dynamic_candidate_cloud` 等调试点云均只反映当前帧，
不会修改静态 PCD，也不会直接替代最终的 `dynamic_cloud`。

在 RViz 中将 `Fixed Frame` 设为 `map`，添加 `Marker` 显示并分别选择上述两个窗口话题即可。
`history_window` 是唯一的动态滑动窗口，表示 base 局部历史障碍的保留边界。
`preprocess_window` 是独立的传感器 FOV/量程诊断包络；`min_z/max_z` 和自体滤除
仍是额外的有效性条件，并未由该线框完整表示。

## 坐标与静态地图要求

静态 PCD、输入点云和 TF 定位必须描述同一个物理地图坐标系。即使输入点云
已经处于 `map` 坐标系，仍必须提供 `sensor_frame`：可见性清除需要真实的
LiDAR 光心，而不能把点云坐标系原点当作传感器位置。

如果 PCD 自身坐标系与 `map` 存在固定偏差，可设置：

```text
static_map_pose: [x, y, z, roll, pitch, yaw]
```

节点会先按 `T_map_pcd = Translation(x,y,z) * Rz(yaw) * Ry(pitch) * Rx(roll)`
把 PCD 转换到 `map`，再建立 KD-Tree。角度单位为弧度；默认单位变换。该参数
解决的是地图的刚性配准误差，不应通过无节制增大 `static_match_radius` 来替代，
否则靠墙的人或箱子也可能被吞掉。

静态 PCD 必须包含 LiDAR 能稳定看到的全部永久表面，例如地面、墙、柱子及
Gazebo 的 ground plane。PCD 中缺失的固定表面会被视为动态候选，这是静态
差分的预期行为，而不是感知模块的语义分类错误。

对于明确存在无限水平地面的场景，可启用可选先验：

```yaml
static_ground_plane_enabled: true
static_ground_plane_z: 0.0
static_ground_plane_tolerance: 0.05
```

满足 `abs(point.z - static_ground_plane_z) <= tolerance` 的点会在逐点差分和
cluster-level 复检中都按静态点处理。该功能默认关闭，坡道、多高度地面或无法
确定绝对地面高度的实机环境不应开启；这些场景应依赖完整 PCD。

当前 Gazebo Building 模型相对源 PCD 在 Z 轴有 `-0.1 m` 固定偏移，且仿真
ground plane 超出 PCD 覆盖范围。因此 Gazebo 启动链加载
`config/dynamic_perception_gazebo.yaml`，其中设置 `static_map_pose` 的 Z 为 `-0.1`
并开启 Z=0 的水平地面先验。基础参数文件仍保持单位变换、地面先验关闭的通用默认值。

`building2_9.pcd` 含有法向量。除直接 KD-tree 半径命中外，节点还会在静态 PCD
邻域内测试点到局部静态表面的法向距离：这只补偿 PCD 的切向采样空隙，不会依据
“墙体/平面”等单独规则删除动态点。默认参数为：

```yaml
static_surface_search_radius: 0.20
static_surface_normal_distance: 0.05
marking_minimum_height: 0.08
marking_height: 0.60
history_window_min_height: 0.08
history_window_max_height: 0.60
```

`marking_minimum_height` / `marking_height` 是 `base_frame` 中当前帧新障碍点的高度门控，
因此顶部墙面和天花板回波不会进入动态候选。`history_window_min_height` /
`history_window_max_height` 独立限定历史体素的保留高度以及 RViz 历史窗口线框，
默认与 marking 范围一致。若修改法向距离阈值，应先在纯静态环境观察
`/dynamic_perception/static_surface_matched_cloud`；不要为消除墙体残留而大幅放宽
该阈值，否则紧贴墙面的薄动态障碍会失去可分辨性。

## 墙体残留的调参顺序

先启动 RViz，同时观察以下三个话题：

```text
/dynamic_perception/current_lidar
/dynamic_perception/static_surface_matched_cloud
/dynamic_perception/dynamic_candidate_cloud
```

低处墙体出现在 `dynamic_candidate_cloud`、但没有出现在
`static_surface_matched_cloud` 时，先确认 `static_map_pose` 与 Gazebo 模型一致。
确认后按以下顺序微调：先将 `static_surface_normal_distance` 从 `0.05` 提升到
`0.06`，仍有残留才提升到 `0.07`；将 `static_cluster_ratio` 设为 `0.50`，使含有
至少一半静态支持点的残留 cluster 被拒绝。这与 DDDMR 的典型
`segmentation_ignore_ratio: 0.5` 目标一致，但本模块按 cluster 中的真实点复检。

例如，可在启动前或通过包含该节点的部署 launch 覆盖同名 ROS 参数：

```yaml
static_surface_normal_distance: 0.07
static_cluster_ratio: 0.50
```

算法阈值统一由 `config/dynamic_perception.yaml` 提供默认值；场景 launch
只覆盖地图、坐标系和已确认的环境配准差异。需要改变算法阈值时，通过 ROS
参数覆盖 YAML 中的同名项，而不是依赖不同 launch 的隐式默认值。

不要把 `static_match_radius` 超过 `0.20m`，也不要把
`static_surface_normal_distance` 超过 `0.08m`。前者会按三维球距离吞掉靠墙物体，
后者会使与墙面距离很小的动态薄板在静态图分辨率内不可区分。

## 与 DDDMR 静态过滤的关系

本模块保留 DDDMR 的 KD-Tree 静态匹配、聚类复检以及 Mark/Clear 思路，但静态
剔除顺序做了增强：先对每个 LiDAR 点执行静态差分，再对剩余点聚类。这样墙边的
人体不会因与墙形成同一聚类而整团误判。复检也查询 cluster 中的实际点，而不是
只查询质心。地图位姿变换和可选地面先验只修正确定的坐标/地图覆盖问题，不改变
动态障碍生命周期。

## 障碍生命周期与安全策略

新聚类首先进入 `TENTATIVE` 状态，连续命中达到 `confirm_hits`（默认 2）后
才成为 `CONFIRMED` 并输出。单帧噪声不会进入最终动态点云。

每个障碍分别保存最新单帧观测和世界坐标体素集合。最新观测只用于 cluster 关联；
最终点云由仍有效的 Mark/Clear 体素生成。机器人改变观察角度或只看到一个侧面时，
新观测会标记对应体素，历史体素不会被单帧 cluster 整体覆盖。

```yaml
obstacle_voxel_size: 0.08
```

模块不再根据可见表面质心位移猜测整个障碍是否移动。该质心会随观察角度变化，
直接用固定距离重置会误删大型静止障碍。移动目标在新位置持续 Mark；旧位置体素
只有在 LiDAR 明确看到该位置已为空时才会逐步 Clear，因此不会无限留下拖影。

已确认障碍采用 DDDMR 风格逐体素清理。对于本帧没有重新 Mark 的历史体素，只要它位于
LiDAR FOV 内、sensor→voxel 射线前方无回波，并且当前完整有效扫描在旧体素邻域无回波，
便在本帧删除。被遮挡、FOV 外或扫描不健康时均保持不变；障碍的全部体素清空后才删除 track。

被前景物遮挡的障碍进入 `OCCLUDED` 并继续输出；离开 FOV、LiDAR 暂时无数据
时，已确认障碍会在当前局部感知窗口内保守保留。扫描不健康时节点保留上一帧有效
快照，仅发布 `scan_healthy=false`。模块不对窗口内的
`CONFIRMED` 或 `OCCLUDED` 障碍使用硬 TTL，避免真实但暂未重新扫描的障碍被错误
释放；离开 `perception_window_size` 后的历史体素不再输出。

仅未输出给规划器的 `TENTATIVE` 候选使用 `max_tentative_age`（默认 2 秒）
进行回收，以防止未确认噪声无限积累；它不会释放已发布的障碍空间。

历史体素仅在当前机器人附近、以 `base_frame` 为中心的 `perception_window_size` 内保留；
离开该局部窗口后不再发布，重新接近时由 LiDAR 重新建立。新动态障碍的 Mark 也只使用
该 base 局部窗口内的点；但射线遮挡与后方回波清除使用完整有效 FOV/量程扫描，避免窗口
边缘外的回波丢失自由空间证据。清除要求完整有效可见性扫描至少包含
`min_clearing_scan_points`（默认 6）个点。点数不足、TF 不可用或点云格式非法时，
节点保留上一帧动态快照，同时在 `/dynamic_perception/scan_healthy` 发布 `false`。

直接清理参数如下：

```yaml
direct_clear_search_radius: 0.10
```

它是旧体素邻域回波的查询半径。此策略优先清理速度：一次有效扫描中的目标邻域漏检
会导致旧体素删除；点云稀疏时应适当增大该值。

## 构建与测试

本工作空间使用 `catkin_make`：

```bash
catkin_make --pkg dynamic_perception_3d
catkin_make run_tests_dynamic_perception_3d
catkin_test_results build/test_results/dynamic_perception_3d
```
