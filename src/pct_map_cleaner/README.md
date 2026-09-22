# PCT 点云地图清洗工具

本目录提供一个 Python 启动器和一个独立的 C++ 点云清洗后端。Python 不执行
点云算法，只负责选择配置文件、查找 `pct_map_cleaner` 可执行文件并转发参数。

后端使用 PCL 读取和写出 PCD，公共类只暴露 STL 和 `Point3f`/`PointCloudXYZ`，
不会把 PCL、FLANN 或 OpenMP 类型泄漏到公共头文件中。最终文件只包含 `x y z`
三个字段。

## 构建

依赖包括 PCL、FLANN、OpenMP 和 yaml-cpp。构建产物固定为 `build/pct_map_cleaner`：

```bash
cd src/pct_map_cleaner
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

## 运行

配置文件默认是脚本同目录下的 `pct_map_cleaner.yaml`，因此可以直接运行：

```bash
cd src/pct_map_cleaner
python3 pct_map_cleaner.py
```

也可以显式指定配置，并覆盖输入输出路径：

```bash
python3 pct_map_cleaner.py \
  --config /path/to/site.yaml \
  --input /path/to/input.pcd \
  --output /path/to/output.pcd
```

配置文件中的相对 `input_pcd` 和 `output_pcd` 路径相对于 YAML 所在目录解释。
命令行优先级为：默认 YAML → `--config` → `--input`/`--output`。

Python 查找后端的顺序为：

1. `PCT_MAP_CLEANER_BIN` 环境变量；
2. `src/pct_map_cleaner/build/pct_map_cleaner`；
3. `PATH` 中的 `pct_map_cleaner`。

后端不存在时，脚本直接报错，不会回退到旧的 Python 点云实现。

C++ 后端也可以直接运行：

```bash
./build/pct_map_cleaner --help
./build/pct_map_cleaner --config pct_map_cleaner.yaml
./build/pct_map_cleaner --config pct_map_cleaner.yaml \
  --input /path/to/input.pcd --output /path/to/output.pcd
```

## 处理流程

固定顺序为：

```text
读取 PCD
  → 删除 NaN/Inf
  → 原始坐标预降采样
  → SOR
  → ROR
  → 小型浮空点团过滤
  → 最终 voxel-center 采样
  → 地面局部空洞补全
  → 写出 XYZ PCD
```

预降采样只减少点数。每个预 voxel 选择距离 voxel 中心最近的原始点，输出坐标
完全来自输入点；代表误差超过 `max_rep_error` 时拒绝继续处理。最终
`uniform_sampling` 才会把每个非空 voxel 表示为 voxel 中心，这是唯一修改 XYZ
坐标的阶段。

SOR 使用 FLANN KD-tree 并行查询，ROR 和浮空点团过滤使用稀疏网格。地面补全只
对最终采样结果工作，根据 XY 邻域、支持方向、支持 cell 数和局部高度层判断小型
缺口，超过 `max_hole_area` 的缺口跳过，不跨楼层或不同高度结构补点。

## 配置要点

新流程相关配置为：

```yaml
pre_sampling:
  enable: true
  spacing: 0.025
  max_rep_error: 0.05
  representative: nearest_to_voxel_center

uniform_sampling:
  enable: true
  spacing: 0.05
  sampling_mode: voxel_center

ground_completion:
  enable: true
  spacing: 0.05
  support_radius: 0.30
  z_tolerance: 0.15
  min_support_directions: 4
  min_support_cells: 4
  max_hole_area: 0.25

performance:
  threads: 0
```

现有的 `sor`、`ror` 和 `floating_cluster` 配置名称继续兼容。`threads: 0` 使用
OpenMP 默认线程数；设置为正数可固定线程数，便于复现实验。

## 验证

```bash
python3 src/pct_map_cleaner/pct_map_cleaner.py --help
python3 src/pct_map_cleaner/pct_map_cleaner.py
(cd src/pct_map_cleaner/build && ctest --output-on-failure)
```

应重点检查报告中的各阶段点数和耗时，并确认输出 PCD 的字段只有 `x y z`。
建议分别用 `threads: 1` 和固定的多线程数运行，比较输出点坐标和点顺序；算法
中的网格、voxel 和排序均使用确定性规则。

## 等价加速与可选 SOR 后端

默认 `sor.search_backend: legacy` 保留原四棵随机 KD-tree、`checks=64` 的
近似邻域语义。预采样复用同一张哈希表完成代表误差验证；ROR 找够邻居即结束
查询；补全复用线程工作区并按支持偏移编号去重；SOR 使用 4096 点分块查询。
滤波参数、阶段顺序、代表点选择和全局统计累加顺序保持不变。

可选配置：

```yaml
sor:
  search_backend: single_exact
```

`single_exact` 使用 FLANN 单棵 KD-tree（leaf size 10、reorder=true、eps=0）。
它不是原近似查询的逐点等价替代。本次真实地图上 SOR 保留集合有明显变化，
因此只作为可选模式提供，不自动切换。改回 `legacy` 即可恢复默认搜索语义。
旧 YAML 缺少该字段时自动使用 `legacy`，未知字段值会报错。

报告新增 `detail.*` 子计时，以及实际 SOR 后端和请求线程数。
`total` 仍只累计七个顶层阶段，不包含 PCD 读写，也不会重复累计子计时。
子计时主要用于定位计算段；局部容器析构等开销仍计入顶层阶段，故子计时之和
不必精确等于该阶段耗时。`threads: 0` 继续遵循 OpenMP 默认线程设置。

### 构建与回归测试

以下命令从本目录执行；进入构建目录运行 CTest，兼容不支持 `--test-dir` 的
旧版 CTest：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j8
(cd build && ctest --output-on-failure)
```

测试包含冻结旧实现的逐点对照、预采样/ROR/补全边界、多高度层、SOR 单点与
分块均值比较、精确 KNN 与暴力计算比较、1/4/8/16 线程一致性，以及配置和
离线工具回归。`BUILD_TESTING=ON` 额外要求 Python 3 解释器，仅使用标准库。
冻结旧版二进制为 `build/pct_map_cleaner_baseline`，不安装为生产入口。

### 真实地图质量与性能对比

工具仅依赖 Python 3 标准库和系统 `/usr/bin/time`。所有输出写入指定的新目录；
目录已存在则拒绝执行，以防覆盖证据或地图。以下命令从本目录执行：

```bash
python3 tools/benchmark.py \
  --baseline build/pct_map_cleaner_baseline \
  --optimized build/pct_map_cleaner \
  --config pct_map_cleaner.yaml \
  --input ../global_pct_planner/rsc/pcd/building2_9.pcd \
  --output-dir /tmp/pct-quality-new --mode quality --threads 1 4 8 16

python3 tools/benchmark.py \
  --baseline build/pct_map_cleaner_baseline \
  --optimized build/pct_map_cleaner \
  --config pct_map_cleaner.yaml \
  --input ../global_pct_planner/rsc/pcd/building2_9.pcd \
  --output-dir /tmp/pct-timing-new --mode timing --threads 1 4 8 16 --repeats 5
```

质量模式用独立进程执行各阶段前缀，比较 packed XYZ 数据哈希（包含点顺序），
并检查各线程数结果一致。`legacy` 的任何差异会使脚本失败；`single_exact` 与
基线的差异会报告但不会被当作等价失败，其跨线程输出仍要求一致。
质量模式继承配置已有开关，不会把原本关闭的浮空点团过滤打开。

性能模式每个版本/线程数组合预热一次，随后每轮各执行一次、共五轮；相邻轮
反转版本顺序，平衡运行顺序影响。每版均有五个样本，不为每个候选重复计算一套
基线。记录阶段时间、总时间、含读写的进程墙钟时间、峰值 RSS 和最终 XYZ 哈希。
`summary.json` 提供中位数、最小/最大值；`equality.json` 检查默认模式一致性。
可重复传 `--variant NAME=/absolute/path/to/binary` 加入增量版本的对比。
配置编辑支持本项目所用的普通顶层 section/两空格子键格式，不支持 YAML 锚点、
内联映射等其他 YAML 表达方式。

几何差异工具使用单树精确最近邻，距离单位为米；它同时导出两边独有的点：

```bash
build/pct_map_cleaner_compare \
  /tmp/pct-quality-new/baseline-t8-ground_completion.pcd \
  /tmp/pct-quality-new/single_exact-t8-ground_completion.pcd \
  /tmp/pct-geometry-new/final 8
```

输出 `final.json`、`final-old-only.pcd`、`final-new-only.pcd`。报告包含双向最近邻
P50/P95/P99/最大值，以及大于 0.025/0.05/0.10 m 的点比例；分位数采用线性插值。
独有点按 XYZ 精确数值比较，以多重集差保留重复点计数。空目标的距离指标为
`null`，并报告 `unmatched_points`。已有输出文件会触发错误，避免覆盖。

最终点数相近、距离分位数较小均不足以单独证明质量相当。正式切换精确后端前，
仍需检查差异点云中的楼梯、栏杆、薄墙和多层地面，并验证下游 PCT 可通行地图。
本次测量和验收结果见 `reports/performance.md`。
