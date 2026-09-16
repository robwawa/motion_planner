# PCT 点云地图清洗工具

`pct_map_cleaner.py` 是一个独立的 Python 3 工具，用于清洗 LIO-SAM
生成的最终 `.pcd` 地图，使其更适合 PCT tomography 使用。

SOR、ROR 和浮空点团阶段只删除原始点。最后的空间降采样阶段会为每个非空
voxel 输出一个代表点，并将该点的 XYZ 放置在 voxel 几何中心，以获得规则的
物理空间分布。工具不进行曲面重建、填补空洞、地面判断，也不修改 PCT
Planner 原代码。

工具位于 `src/pct_map_cleaner/`，不是 ROS 或 Catkin 软件包，也不需要 ROS
依赖。

## 依赖安装

```bash
pip install open3d numpy pyyaml
```

## 运行方式

进入工具目录后运行：

```bash
cd /home/wa/inspection/3D_motion_planner/motion_planner/src/pct_map_cleaner

python3 pct_map_cleaner.py \
    --config pct_map_cleaner.yaml
```

也可以通过命令行覆盖 YAML 中的输入和输出路径：

```bash
python3 pct_map_cleaner.py \
    --config pct_map_cleaner.yaml \
    --input /path/to/GlobalMap.pcd \
    --output /path/to/GlobalMap_pct_clean.pcd
```

命令行中的 `--input` 和 `--output` 优先级高于 YAML 配置。

查看所有命令行参数：

```bash
python3 pct_map_cleaner.py --help
```

示例 YAML 中的输入和输出路径是占位路径。首次使用时请修改配置，或者直接
通过 `--input` 和 `--output` 提供路径。

## 与 PCT 分辨率的关系

当前项目的 PCT 配置位于：

```text
src/pct_planner/config/pct_planner.yaml
```

其中当前地图参数为：

```yaml
pct:
  map:
    resolution: 0.10
    slice_dh: 0.5
```

PCT tomography 在 XY 平面使用 `0.10 m` 分辨率将点映射到地图栅格。因此本
工具默认使用：

```yaml
uniform_sampling:
  spacing: 0.05
```

也就是：

```text
uniform_sampling.spacing = 0.5 * PCT map.resolution
```

这样可以在进入 PCT 前降低点云冗余，同时使采样间距小于 PCT 的 XY 栅格分辨率。
这里的“均匀”指空间密度由固定 voxel 尺寸控制，不代表所有输出点之间严格等距。

`spacing` 当前是手动固定值，不会在运行时自动读取 PCT YAML。如果以后修改了
PCT 的 `map.resolution`，请同步调整清洗配置中的 `spacing`，建议继续保持
`spacing < resolution`，并优先使用约 `0.5 * resolution` 的保守值。

PCT 的 `slice_dh=0.5 m` 只用于 tomography 的垂直分层。本工具不使用它删除、
合并或判断 Z 方向点，也不会据此删除楼梯、坡道、顶棚、楼板、悬空平台或其他
多层结构。

推荐工作流：

```text
GlobalMap.pcd
    ↓
pct_map_cleaner.py
    ↓
GlobalMap_pct_clean.pcd
    ↓
PCT tomography
    ↓
tomogram
    ↓
PCT Planner
```

## 处理流程

处理顺序如下：

1. 使用 `open3d.io.read_point_cloud()` 读取 PCD。
2. 删除 XYZ 中包含 NaN、Inf 或 `-Inf` 的点。
3. 根据配置执行 Statistical Outlier Removal（SOR）。
4. 根据配置执行 Radius Outlier Removal（ROR）。
5. 使用 DBSCAN 分析点团，只删除满足条件的小型浮空点团。
6. 使用 NumPy 执行 3D voxel 空间均匀采样。
7. 保存 `GlobalMap_pct_clean.pcd`。

点云包围盒只用于统计和日志输出，不用于裁剪地图范围。DBSCAN 的
`label == -1` 点只会被统计，不会被自动删除，因为它们可能是栏杆、细边缘、
楼梯棱边或其他合法的稀疏结构。

## 参数说明

### SOR

- `nb_neighbors`：统计邻域点数量。
- `std_ratio`：统计异常阈值。
- `std_ratio` 越小，删除越激进，越可能误删真实稀疏结构。
- SOR 删除比例超过 20% 时，程序会打印 warning。

默认值：

```yaml
sor:
  enable: true
  nb_neighbors: 30
  std_ratio: 2.5
```

### ROR

- `radius`：邻域搜索半径。半径太小，稀疏真实结构越容易被误删。
- `nb_points`：半径内所需的邻居点数量。该值越大，过滤越激进。
- ROR 删除比例超过 20% 时，程序会打印 warning。

默认值：

```yaml
ror:
  enable: true
  nb_points: 2
  radius: 0.15
```

#### 增强孤立点去除

如果运行后仍有明显的孤立飞点，建议优先逐步增大 `nb_points`：

```yaml
ror:
  enable: true
  nb_points: 3
  radius: 0.15
```

建议按以下顺序测试：

```text
nb_points: 2 → 3 → 4
```

`nb_points` 越大，一个点需要更多邻居才能保留，ROR 会更加激进。不要为了
删除孤立点而盲目增大 `radius`；半径过大可能让原本稀疏的点找到较远邻居而被
保留，具体值应结合原始点云密度调整。半径过小则更容易误删栏杆、细杆、楼梯
边缘和远处稀疏结构。

如果仍有统计异常点，可以再适度降低 SOR 的 `std_ratio`：

```yaml
sor:
  nb_neighbors: 30
  std_ratio: 2.0
```

推荐从 `2.5` 调整到 `2.0`，不要一开始就使用过小值。`std_ratio` 越小，越容易
误删楼梯边缘、栏杆、细杆、稀疏墙面和顶棚边缘。

如果问题不是单个孤立点，而是小型密集浮空点团，可以调整：

```yaml
floating_cluster:
  remove_if_point_count_below: 50
```

当前默认值为 `30`。只有确认仍存在小型浮空点团时，才建议谨慎增大
`max_bbox_x`、`max_bbox_y` 和 `max_bbox_z`，因为阈值过大可能误删真实小物体或
合法的小型结构。

推荐调参顺序为：

1. 将 `ror.nb_points` 从 `2` 调整为 `3`。
2. 检查删除比例和 CloudCompare 中的 `removed_ror.pcd`。
3. 仍有孤立点时，再尝试 `nb_points: 4`。
4. 最后将 `sor.std_ratio` 从 `2.5` 调整为 `2.0`。
5. 对小型密集浮空点团，再考虑提高 `remove_if_point_count_below`。

不要一次大幅修改多个参数。SOR 或 ROR 删除比例超过 20% 时，必须重点检查
楼梯、栏杆、悬空平台、顶棚边缘和上下多层结构是否被误删。

### 小型浮空点团

该阶段使用 `cluster_dbscan()`。对于每一个 `label >= 0` 的 cluster，计算点数
和 XYZ 三个方向的包围盒尺寸。

只有同时满足以下条件时才会删除：

```text
point_count < remove_if_point_count_below
dx < max_bbox_x
dy < max_bbox_y
dz < max_bbox_z
```

程序不会只保留最大 cluster，也不会删除所有与主体不连接的 cluster。因此，
顶棚、楼板、栏杆、悬空平台、桥梁、overhang 以及上下多层结构不会因为属于
独立连通组件而被自动删除。

注意：

- `eps` 过小，可能将一个真实结构拆成多个 cluster。
- 浮空 cluster 的 bbox 阈值过大，可能误删真实小物体或小型结构。
- 该阶段不判断一个结构是否是地面、顶棚、楼梯、坡道或可通行区域。

### 空间均匀采样

工具没有使用 `uniform_down_sample()`，因为它只是按照点数组顺序抽点，
并不保证空间均匀。

当前采样模式为 `voxel_center`，不是 voxel centroid。具体实现为：

1. 以当前点云最小坐标作为整体网格原点。
2. 使用 NumPy 向量化计算每个点的 voxel 索引。
3. 使用排序和分组处理每个 voxel。
4. 计算 voxel 中心。
5. 每个非空 voxel 输出一个位于几何中心的代表点。
6. 使用 voxel 内距离中心最近的原始点同步 colors 和 normals。

因此，输出点的物理位置会被规则化到 voxel 中心，但不会计算 voxel centroid、
平均值、中位数、插值或曲面重建。这个阶段允许代表点的 XYZ 偏离原始点位置。

单个原始点到其 voxel 中心的最大理论位移为：

```text
sqrt(3) / 2 * spacing
```

当 `spacing=0.05 m` 时，最大理论位移约为 `0.0433 m`。程序会在日志中输出：

```text
occupied voxels:
max displacement:
mean displacement:
```

由于薄墙、栏杆、楼梯棱边和悬空结构可能被移动到 voxel 中心，使用清洗后的
地图生成 PCT tomogram 后，应在 CloudCompare 和 PCT 输出中检查几何偏移。

`spacing` 是 voxel 边长。值越大，地图越稀疏，也可能影响 PCT tomography
的连续性。

如果：

```text
PCT map.resolution = 0.10 m
```

建议第一轮测试保持：

```yaml
uniform_sampling:
  sampling_mode: voxel_center
  spacing: 0.05
```

即 `spacing < PCT resolution`。

本工具的默认配置已经按照上述关系设置为 `spacing: 0.05`。降采样只负责控制
输入点云的空间密度，PCT tomography 仍负责 XY 栅格映射、垂直切片以及后续的
多层结构处理。

## 保护 PCT 多层结构

PCT Planner 负责处理多层结构，因此本工具不尝试判断：

- 什么是地面；
- 什么是顶棚；
- 哪里是楼梯或坡道；
- 哪些结构可通行；
- 哪个 cluster 是地图主体。

清洗流程只处理统计异常点、真正孤立的飞点、小型密集浮空点团以及空间冗余
点，不做地图理解。

## 与 PCT tomography 的验证

生成清洗地图后，可使用该 PCD 作为 PCT tomography 的输入：

```bash
cd /home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/tomography/scripts

python3 tomography.py --backend cpu \
    --pcd-file /path/to/GlobalMap_pct_clean.pcd \
    --tomogram-name GlobalMap_pct_clean
```

验证时应检查：

- 清洗后的 PCD 能被 PCT 正常加载；
- tomography 能正常生成 tomogram；
- 输出仍包含预期的多个高度层；
- 楼梯、坡道、栏杆、悬空平台、overhang 和上下多层结构没有因清洗脚本的
  最大连通组件假设而被删除；
- 点数减少后，PCT 的 XY 栅格和层析结果仍保持连续性。

上述命令只验证 PCT 对清洗地图的兼容性，不会修改 PCT Planner 或 tomography
的源代码。

## Debug 中间文件

配置：

```yaml
debug:
  save_intermediate: true
  output_directory: "./pct_map_cleaner_debug"
```

开启后会保存：

```text
00_input_valid.pcd
01_sor.pcd
removed_sor.pcd
02_ror.pcd
removed_ror.pcd
03_cluster_clean.pcd
removed_floating_clusters.pcd
04_uniform.pcd
```

其中：

- `removed_sor.pcd` 保存 SOR 删除的点；
- `removed_ror.pcd` 保存 ROR 删除的点；
- `removed_floating_clusters.pcd` 保存被小型浮空 cluster 阶段删除的点。

这些文件可以使用 CloudCompare 或 Open3D 检查。

如果某个处理模块关闭，程序会跳过该处理，并在对应中间文件中保存未改变的
点云结果。SOR、ROR 删除点文件仍会生成；浮空 cluster 删除文件受
`save_removed_clusters` 控制。

## 点属性同步

对于 Open3D 支持的 `colors` 和 `normals`，程序会使用与 XYZ 相同的点索引进行
同步保留。

当前版本主要保证 XYZ 几何正确性。对于 Open3D 不支持或无法可靠同步的其他
PCD 字段，不进行重建或猜测。

## 性能与日志

程序使用 `time.perf_counter()` 统计并打印 SOR、ROR、DBSCAN、空间采样和总耗时。

百万级点云处理时：

- SOR 和 ROR 的邻域搜索可能占用较多时间；
- DBSCAN 通常是最主要的性能瓶颈；
- 空间采样主要使用 NumPy 向量化，不对每个点执行 Python KD-tree 查询；
- 空间采样会创建 voxel 索引、排序和距离临时数组，需要足够内存。

空间采样是主动降采样，因此即使减少比例超过 20%，也不会打印异常 warning。
