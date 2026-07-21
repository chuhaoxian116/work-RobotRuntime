#include "robot_runtime/robot_runtime.h"

#include <cia402/cia402.h>

namespace robot_runtime {

namespace {

// 将 RobotRuntime 公共反馈和命令字段组装成 CiA402 单周期调用数据。
cia402::AxisData MakeCia402Data(const AxisRuntimeData& axis) {
  cia402::AxisData data{};
  data.inData.statusword = axis.feedback.statusword;
  data.inData.mode_display = axis.feedback.mode_display;
  data.outData.controlword = axis.command.controlword;
  data.outData.mode = axis.command.mode;
  return data;
}

// CiA402 接口只生成控制字和目标模式，其它运动命令保持由上层算法维护。
void UpdateAxisCommand(const cia402::AxisData& data, AxisRuntimeData& axis) {
  axis.command.controlword = data.outData.controlword;
  axis.command.mode = data.outData.mode;
}

// 只有完成配置且本周期通信有效的轴，才允许执行 CiA402 状态控制。
bool IsAxisAvailable(const AxisRuntimeData& axis) {
  return axis.configured != 0 && axis.communication_valid != 0;
}

// 对单轴执行使能或断使能，并在调用 CiA402 前检查轴是否可用。
cia402::FbStatus PowerRobotAxis(AxisRuntimeData& axis, bool enable) {
  if (!IsAxisAvailable(axis)) {
    return cia402::FbStatus::kError;
  }

  cia402::AxisData data = MakeCia402Data(axis);
  const cia402::FbStatus status = cia402::PowerAxis(data, enable);
  UpdateAxisCommand(data, axis);
  return status;
}

// 对单轴请求目标运行模式，实际完成状态由 0x6061 模式反馈决定。
cia402::FbStatus SwitchRobotAxisMode(AxisRuntimeData& axis, cia402::AxisMode target_mode) {
  if (!IsAxisAvailable(axis)) {
    return cia402::FbStatus::kError;
  }

  cia402::AxisData data = MakeCia402Data(axis);
  const cia402::FbStatus status = cia402::SwitchMode(data, target_mode);
  UpdateAxisCommand(data, axis);
  return status;
}

// 对单轴执行回零控制，使能状态要求由调用方通过参数决定。
cia402::FbStatus HomeRobotAxis(AxisRuntimeData& axis, bool start, bool require_operation_enabled) {
  if (!IsAxisAvailable(axis)) {
    return cia402::FbStatus::kError;
  }

  cia402::AxisData data = MakeCia402Data(axis);
  const cia402::FbStatus status = cia402::Homing(data, start, require_operation_enabled);
  UpdateAxisCommand(data, axis);
  return status;
}

// 对单轴执行或撤销 CiA402 清错，不在本层判断驱动器故障是否已经消失。
cia402::FbStatus ClearRobotAxisError(AxisRuntimeData& axis, bool execute) {
  if (!IsAxisAvailable(axis)) {
    return cia402::FbStatus::kError;
  }

  cia402::AxisData data = MakeCia402Data(axis);
  const cia402::FbStatus status = cia402::ClearAxisError(data, execute);
  UpdateAxisCommand(data, axis);
  return status;
}

}  // namespace

// 每周期处理所有机器人本体轴，并在全部轴更新后汇总整机使能结果。
RuntimeResult PowerRobot(RobotRuntimeData& robot, bool enable) {
  if (robot.robot_axis_count > kMaxRobotAxisCount) {
    return RuntimeResult::kError;
  }

  bool has_error = false;
  for (uint8_t i = 0; i < robot.robot_axis_count; ++i) {
    const cia402::FbStatus status = PowerRobotAxis(robot.robot_axes[i], enable);
    if (status == cia402::FbStatus::kError) {
      has_error = true;
    }
  }

  return has_error ? RuntimeResult::kError : RuntimeResult::kSuccess;
}

// 每周期为所有机器人本体轴请求相同模式，并汇总模式切换结果。
RuntimeResult SwitchRobotMode(RobotRuntimeData& robot, int8_t target_mode) {
  if (robot.robot_axis_count > kMaxRobotAxisCount) {
    return RuntimeResult::kError;
  }

  bool has_error = false;
  for (uint8_t i = 0; i < robot.robot_axis_count; ++i) {
    const cia402::FbStatus status =
        SwitchRobotAxisMode(robot.robot_axes[i], static_cast<cia402::AxisMode>(target_mode));
    if (status == cia402::FbStatus::kError) {
      has_error = true;
    }
  }

  return has_error ? RuntimeResult::kError : RuntimeResult::kSuccess;
}

// 每周期处理所有机器人本体轴的回零状态，不负责回零前后的模式切换。
RuntimeResult HomeRobot(RobotRuntimeData& robot, bool start, bool require_operation_enabled) {
  if (robot.robot_axis_count > kMaxRobotAxisCount) {
    return RuntimeResult::kError;
  }

  bool has_error = false;
  for (uint8_t i = 0; i < robot.robot_axis_count; ++i) {
    const cia402::FbStatus status =
        HomeRobotAxis(robot.robot_axes[i], start, require_operation_enabled);
    if (status == cia402::FbStatus::kError) {
      has_error = true;
    }
  }

  return has_error ? RuntimeResult::kError : RuntimeResult::kSuccess;
}

// 每周期处理所有机器人本体轴的清错请求，并汇总本周期执行结果。
RuntimeResult ClearRobotError(RobotRuntimeData& robot, bool execute) {
  if (robot.robot_axis_count > kMaxRobotAxisCount) {
    return RuntimeResult::kError;
  }

  bool has_error = false;
  for (uint8_t i = 0; i < robot.robot_axis_count; ++i) {
    const cia402::FbStatus status = ClearRobotAxisError(robot.robot_axes[i], execute);
    if (status == cia402::FbStatus::kError) {
      has_error = true;
    }
  }

  return has_error ? RuntimeResult::kError : RuntimeResult::kSuccess;
}

}  // namespace robot_runtime
