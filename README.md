# RobotRuntime

RobotRuntime 是面向多轴机器人实时周期调用的通用业务库工程。

当前提供多轴机器人运行时数据、整机使能、模式切换、回零和清错接口，
内部通过 CiA402 SDK 对各机器人本体轴执行单周期状态控制。

工程包含三个独立子仓库：

- `third_party/cia402`：CiA402 SDK 依赖；其 `AxisData` 是 IgH 与
  RobotRuntime 共用的驱动状态和命令类型；
- `third_party/robot_interface`：IgH、RobotRuntime 和算法共用的机器人数据契约；
- `sdk`：RobotRuntime 自己的 SDK 交付仓库，后续输出公开头文件和动态库。

## 工程结构

```text
work-RobotRuntime/
  CMakeLists.txt
  NAMING_STYLE.md
  .clang-format
  .gitignore
  src/
  examples/
  third_party/
    cia402/                   # work-CiA402-SDK 子仓库
      include/cia402/
      lib/
    robot_interface/          # work-RobotInterface-SDK 子仓库
      include/robot_data.h
  sdk/                        # work-RobotRuntime-SDK 子仓库
    include/robot_runtime/
      robot_runtime.h
    lib/
    README.md
```

命名、注释和格式化规则统一参考 `NAMING_STYLE.md`。

## 配置检查

首次克隆主仓库后先初始化两个子模块：

```bash
git submodule update --init --recursive
```

然后执行 CMake 配置检查：

```bash
cmake -S . -B build
cmake --build build
```

当前 CMake 已导入 `cia402::cia402` 依赖，并使用以下 RobotRuntime 交付路径：

```text
公开头文件：sdk/include/robot_runtime/robot_runtime.h
动态库目录：sdk/lib/
```

`robot_runtime.h` 依赖 `work-RobotInterface-SDK` 提供的
`robot_data.h`。使用 RobotRuntime SDK 时，调用方还应提供同版本
RobotInterface SDK 和 CiA402 SDK 的 `include` 路径。

构建生成 `librobot_runtime.so` 及其版本软链接。当前尚未提供示例程序。
