# motion_planner 代码编写规范

本文档是 `motion_planner` 工程新增和修改代码时的统一规则。工程已有代码逐步迁移，修改旧代码时应遵循本规范；不相关的历史代码和第三方代码不要求一次性重写。

## 1. 基本原则

- 优先保证行为正确、线程安全和可维护性，再进行性能优化。
- 一个函数只负责一个清晰的职责；复杂流程拆分为可测试的小函数。
- 修改应尽量局部化，不顺带格式化无关文件。
- 不复制第三方源码，不修改自动生成代码，不覆盖用户已有修改。
- 所有新增功能必须同时考虑构建、安装、运行时资源和测试。
- 代码、注释和提交说明应能说明“为什么这样做”，而不只描述“做了什么”。

## 2. C++ 规范

### 2.1 编译标准

- 使用 C++17；不得引入依赖更高版本特性的代码，除非同步更新工程构建要求。
- 编译配置保持 Release、`-O2 -Wall -pthread`；禁止在单个源文件中私自覆盖全局编译选项。
- 生产代码不得使用未定义行为、裸的资源所有权或依赖初始化顺序的全局对象。

### 2.2 命名与格式

- 类型使用 `PascalCase`，函数和变量使用 `snake_case`，常量使用 `kPascalCase`。
- 成员变量使用工程现有后缀风格，例如 `planner_mutex_`。
- 头文件使用 include guard 或 `#pragma once`，并按“本类头文件、标准库、第三方库、ROS、项目头文件”分组。
- 优先使用 `const`、引用、`enum class`、`std::unique_ptr` 和 `std::shared_ptr`。
- 避免隐式窄化转换；跨类型转换应明确说明意图。
- 公开接口的参数、返回值和异常行为必须清晰，必要时添加 Doxygen 注释。
- 不在头文件中定义非 `inline` 的普通函数或可变全局变量。

### 2.3 错误处理

- 可恢复错误返回错误码、`false` 或异常，并由上层决定处理方式。
- 不使用 `FATAL` 代替普通错误；只有进程无法继续或全局不变量破坏时使用 `FATAL`。
- 输入、参数、文件和消息数据必须校验边界、空值、长度和有限性。
- 捕获异常时保留原始上下文，禁止静默吞掉异常。

### 2.4 性能与线程

- 高频回调中禁止无界内存增长、重复创建重量对象和无条件输出日志。
- 共享数据必须明确互斥、快照或所有权规则；锁的范围应尽量短。
- 不在持锁期间执行可能阻塞的 ROS、文件或外部库调用，除非有明确理由。
- 算法耗时、迭代细节和大块矩阵默认使用 DEBUG，不输出到标准输出。

## 3. ROS 节点规范

- 节点启动顺序固定为：`ros::init()`、日志初始化、参数读取与校验、资源加载、通信接口创建、业务对象启动。
- 节点启动完成只输出一条 INFO，包含真正影响运行的关键参数、topic、frame 和资源路径。
- topic、frame、参数名和 Action/Service 名称不得在多个地方无序硬编码；可配置内容优先放入 launch/config。
- 回调函数只做消息校验、状态更新和任务调度；复杂计算放入独立业务类。
- 关闭节点时释放线程、定时器、文件和外部资源，不依赖析构顺序完成关键业务动作。
- 消息、Action 和 Service 变更必须同步更新生成依赖、运行依赖、测试和 install 规则。

## 4. 日志规范

### 4.1 统一接口

C++ 运行代码统一使用 `motion_planner_log`：

```cpp
#include <motion_planner_log/logging.h>

motion_planner_log::initialize("package_name", "module_name", argv[0]);
MOTION_PLANNER_LOG_INFO("Ready: frame=%s", frame.c_str());
```

- 禁止在一方运行代码中直接使用 `ROS_INFO/WARN/ERROR/DEBUG/FATAL`。
- 禁止使用业务性的 `printf`、`std::cout`、`std::cerr` 输出运行日志。
- 不在消息中重复手写模块名、节点名或无意义的 `[MODULE]` 前缀。
- Python 节点使用 `motion_planner_log.configure(package_name, module_name)` 返回的标准 logger。

### 4.2 级别选择

- `INFO`：启动完成、关键配置、资源加载成功、状态切换和正常恢复。
- `WARNING`：输入被忽略、参数回退、暂时缺少数据、进入降级模式和可恢复异常。
- `ERROR`：当前请求或当前功能失败，但进程可以继续运行。
- `FATAL`：进程无法继续，或继续运行必然产生错误结果。
- `DEBUG`：高频成功路径、算法中间量、耗时、点数和开发诊断信息。

循环回调中的重复 WARNING/ERROR 必须使用 `*_THROTTLE` 或 `*_ONCE`。日志消息应包含可检索的对象 ID、错误码、资源路径或状态值。

## 5. CMakeLists.txt 规范

统一按以下顺序组织，并为每一段添加清楚的中文注释：

1. `cmake_minimum_required` 和 `project`；
2. 编译选项；
3. ROS 与系统依赖；
4. Message/Service/Action 生成；
5. `catkin_package` 导出；
6. 库和可执行文件；
7. 安装规则；
8. 测试与运行时 RPATH。

规则：

- `find_package`、`catkin_package` 和 `package.xml` 必须保持一致。
- 使用 `${CATKIN_PACKAGE_BIN_DESTINATION}`、`${CATKIN_PACKAGE_LIB_DESTINATION}`、`${CATKIN_PACKAGE_INCLUDE_DESTINATION}` 和 `${CATKIN_PACKAGE_SHARE_DESTINATION}`，禁止写死 install 路径。
- 每个可执行文件、库和消息生成目标声明必要的 `add_dependencies`。
- launch、config、resource、model、world、Python 节点和运行库必须有明确 install 规则。
- 第三方依赖通过可配置路径查找，不写死个人机器目录。
- 私有第三方库需要 RPATH 时，说明来源和原因；不得把第三方源码或构建目录复制进 install。

## 6. package.xml 规范

采用工程统一格式：

```xml
<?xml version="1.0"?>
<package>
```

- 编译依赖使用 `build_depend`，运行依赖使用 `run_depend`。
- 消息包声明 `message_generation` 和 `message_runtime`。
- 测试仅声明必要的 `test_depend`。
- 依赖应真实使用，删除重复、无效和模板化依赖。
- 新增项目日志功能包时声明 `motion_planner_log` 的编译和运行依赖。

## 7. Python 规范

- 使用 Python 3，文件开头声明 `#!/usr/bin/env python3`。
- 模块、函数和变量使用 `snake_case`；类使用 `PascalCase`。
- 节点入口使用 `main()`，并用 `if __name__ == "__main__":` 调用。
- 业务日志使用标准 logger，不使用散落的 `print()`；用户交互、帮助文本和纯数据导出可以保留 `print()`。
- 异常处理必须保留原有控制流和上下文；高频循环日志必须限频。
- Python 节点必须通过 CMake 安装，并验证从 install 空间导入和运行。

## 8. 测试规范

- 新增公共类、数据格式、参数校验和状态机逻辑时必须增加单元测试。
- 测试不得依赖开发者个人目录、未声明环境变量或未安装的外部资源。
- 文件、地图、模型等 fixture 必须随包管理或在测试中明确检查缺失原因。
- 算法回归测试应固定输入、输出关键字段和容差；不要用不稳定的耗时值作为唯一断言。
- 提交前至少执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j4 install
source install/setup.bash
git diff --check
```

## 9. 第三方与自动生成代码边界

以下目录默认不直接修改：

- `third_party/`
- `pct_planner/planner/lib/3rdparty/`
- `mpc_generated/`
- `local_sensing/include/glm/`
- `third_party/acados/`
- ROS 生成的 `devel/`、`build/` 和消息生成文件。

需要改变第三方行为时，应在项目自己的适配层、配置或 CMake 中处理，并在代码审查说明原因。

## 10. 提交前检查清单

- [ ] 未修改无关文件、第三方代码和自动生成代码。
- [ ] 新增依赖已同步到 CMake、`catkin_package` 和 `package.xml`。
- [ ] 新增节点已初始化项目日志，并包含启动摘要。
- [ ] 没有直接使用 ROS 日志宏、业务 `printf/cout/cerr` 或彩色 ANSI 控制字符。
- [ ] 高频日志已使用 DEBUG、ONCE 或 THROTTLE。
- [ ] 资源、脚本、库、消息和 launch 均可安装。
- [ ] 单元测试和必要的回归测试通过。
- [ ] `catkin_make -j4 install` 通过，且 `build/`、`devel/`、`install/` 均存在。
- [ ] `git diff --check` 通过。
