#include "robot_runtime/robot_runtime.h"

namespace robot_runtime {

namespace {

// 只有完成配置且本周期通信有效的轴，才允许执行 CiA402 状态控制。
bool IsAxisAvailable(const AxisRuntimeData& axis) {
  return axis.configured != 0 && axis.communication_valid != 0;
}

// 对单轴执行使能或断使能，并在调用 CiA402 前检查轴是否可用。
cia402::FbStatus PowerRobotAxis(AxisRuntimeData& axis, bool enable) {
  if (!IsAxisAvailable(axis)) {
    return cia402::FbStatus::kError;
  }

  return cia402::PowerAxis(axis.cia402_data, enable);
}

// 对单轴请求目标运行模式，实际完成状态由 0x6061 模式反馈决定。
cia402::FbStatus SwitchRobotAxisMode(AxisRuntimeData& axis, cia402::AxisMode target_mode) {
  if (!IsAxisAvailable(axis)) {
    return cia402::FbStatus::kError;
  }

  return cia402::SwitchMode(axis.cia402_data, target_mode);
}

// 对单轴执行回零控制，使能状态要求由调用方通过参数决定。
cia402::FbStatus HomeRobotAxis(AxisRuntimeData& axis, bool start, bool require_operation_enabled) {
  if (!IsAxisAvailable(axis)) {
    return cia402::FbStatus::kError;
  }

  return cia402::Homing(axis.cia402_data, start, require_operation_enabled);
}

// 对单轴执行或撤销 CiA402 清错，不在本层判断驱动器故障是否已经消失。
cia402::FbStatus ClearRobotAxisError(AxisRuntimeData& axis, bool execute) {
  if (!IsAxisAvailable(axis)) {
    return cia402::FbStatus::kError;
  }

  return cia402::ClearAxisError(axis.cia402_data, execute);
}

}  // namespace

// 每周期处理所有机器人本体轴，并在全部轴更新后汇总整机使能结果。
cia402::FbStatus PowerRobot(RobotRuntimeData& robot, bool enable) {
  if (robot.robot_axis_count > kMaxRobotAxisCount) {
    return cia402::FbStatus::kError;
  }

  bool has_error = false;
  bool has_busy = false;
  for (uint8_t i = 0; i < robot.robot_axis_count; ++i) {
    const cia402::FbStatus status = PowerRobotAxis(robot.robot_axes[i], enable);
    if (status == cia402::FbStatus::kError) {
      has_error = true;
    } else if (status == cia402::FbStatus::kBusy) {
      has_busy = true;
    }
  }

  if (has_error) {
    return cia402::FbStatus::kError;
  }
  if (has_busy) {
    return cia402::FbStatus::kBusy;
  }
  return cia402::FbStatus::kDone;
}

// 每周期为所有机器人本体轴请求相同模式，并汇总模式切换结果。
cia402::FbStatus SwitchRobotMode(RobotRuntimeData& robot, cia402::AxisMode target_mode) {
  if (robot.robot_axis_count > kMaxRobotAxisCount) {
    return cia402::FbStatus::kError;
  }

  bool has_error = false;
  bool has_busy = false;
  for (uint8_t i = 0; i < robot.robot_axis_count; ++i) {
    const cia402::FbStatus status = SwitchRobotAxisMode(robot.robot_axes[i], target_mode);
    if (status == cia402::FbStatus::kError) {
      has_error = true;
    } else if (status == cia402::FbStatus::kBusy) {
      has_busy = true;
    }
  }

  if (has_error) {
    return cia402::FbStatus::kError;
  }
  if (has_busy) {
    return cia402::FbStatus::kBusy;
  }
  return cia402::FbStatus::kDone;
}

// 每周期处理所有机器人本体轴的回零状态，不负责回零前后的模式切换。
cia402::FbStatus HomeRobot(RobotRuntimeData& robot, bool start, bool require_operation_enabled) {
  if (robot.robot_axis_count > kMaxRobotAxisCount) {
    return cia402::FbStatus::kError;
  }

  bool has_error = false;
  bool has_busy = false;
  for (uint8_t i = 0; i < robot.robot_axis_count; ++i) {
    const cia402::FbStatus status =
        HomeRobotAxis(robot.robot_axes[i], start, require_operation_enabled);
    if (status == cia402::FbStatus::kError) {
      has_error = true;
    } else if (status == cia402::FbStatus::kBusy) {
      has_busy = true;
    }
  }

  if (has_error) {
    return cia402::FbStatus::kError;
  }
  if (has_busy) {
    return cia402::FbStatus::kBusy;
  }
  return cia402::FbStatus::kDone;
}

// 每周期处理所有机器人本体轴的清错请求，并汇总本周期执行结果。
cia402::FbStatus ClearRobotError(RobotRuntimeData& robot, bool execute) {
  if (robot.robot_axis_count > kMaxRobotAxisCount) {
    return cia402::FbStatus::kError;
  }

  bool has_error = false;
  bool has_busy = false;
  for (uint8_t i = 0; i < robot.robot_axis_count; ++i) {
    const cia402::FbStatus status = ClearRobotAxisError(robot.robot_axes[i], execute);
    if (status == cia402::FbStatus::kError) {
      has_error = true;
    } else if (status == cia402::FbStatus::kBusy) {
      has_busy = true;
    }
  }

  if (has_error) {
    return cia402::FbStatus::kError;
  }
  if (has_busy) {
    return cia402::FbStatus::kBusy;
  }
  return cia402::FbStatus::kDone;
}

}  // namespace robot_runtime
