# RobotRuntime

RobotRuntime 是面向多轴机器人实时周期调用的通用业务库工程。

当前阶段只建立工程框架，不包含公开 API、业务变量、CiA402 封装或运行时实现。

工程包含两个独立子仓库：

- `third_party/cia402`：CiA402 SDK 依赖，只读取公开头文件和动态库；
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

当前 CMake 已导入 `cia402::cia402` 依赖，并预留以下 RobotRuntime 交付路径：

```text
公开头文件：sdk/include/robot_runtime/robot_runtime.h
动态库目录：sdk/lib/
```

当前不会生成 RobotRuntime 动态库或示例程序，相关 Target 将在接口设计确定后接入。
