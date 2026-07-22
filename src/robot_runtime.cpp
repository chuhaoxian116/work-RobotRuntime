#include "robot_runtime/robot_runtime.h"

#include <cia402/cia402.h>

namespace robot_runtime {

namespace {

// robot_axis_count 表示已配置的本体轴数；通信有效性由 IgH 每周期写入公共反馈。
bool IsAxisAvailable(const robot_interface::AxisFeedback& feedback) {
  return feedback.communication_valid != 0;
}

// 对单轴执行使能或断使能，并在调用 CiA402 前检查轴是否可用。
cia402::FbStatus PowerRobotAxis(cia402::AxisData& axis,
                                const robot_interface::AxisFeedback& feedback,
                                bool enable) {
  if (!IsAxisAvailable(feedback)) {
    return cia402::FbStatus::kError;
  }

  return cia402::PowerAxis(axis, enable);
}

// 对单轴请求目标运行模式，实际完成状态由 0x6061 模式反馈决定。
cia402::FbStatus SwitchRobotAxisMode(
    cia402::AxisData& axis,
    const robot_interface::AxisFeedback& feedback,
    cia402::AxisMode target_mode) {
  if (!IsAxisAvailable(feedback)) {
    return cia402::FbStatus::kError;
  }

  return cia402::SwitchMode(axis, target_mode);
}

// 对单轴执行回零控制，使能状态要求由调用方通过参数决定。
cia402::FbStatus HomeRobotAxis(cia402::AxisData& axis,
                               const robot_interface::AxisFeedback& feedback,
                               bool start,
                               bool require_operation_enabled) {
  if (!IsAxisAvailable(feedback)) {
    return cia402::FbStatus::kError;
  }

  return cia402::Homing(axis, start, require_operation_enabled);
}

// 对单轴执行或撤销 CiA402 清错，不在本层判断驱动器故障是否已经消失。
cia402::FbStatus ClearRobotAxisError(
    cia402::AxisData& axis,
    const robot_interface::AxisFeedback& feedback,
    bool execute) {
  if (!IsAxisAvailable(feedback)) {
    return cia402::FbStatus::kError;
  }

  return cia402::ClearAxisError(axis, execute);
}

}  // namespace

// 每周期根据公共服务请求处理所有机器人本体轴，并汇总整机使能结果。
RuntimeResult PowerRobot(RobotRuntimeData& robot) {
  if (robot.cycle_data.service.power_request_valid == 0) {
    return RuntimeResult::kSuccess;
  }

  if (robot.cycle_data.robot_axis_count > robot_interface::kMaxRobotAxisCount) {
    return RuntimeResult::kError;
  }

  const bool enable = robot.cycle_data.service.power_enable != 0;
  bool has_error = false;
  for (uint8_t i = 0; i < robot.cycle_data.robot_axis_count; ++i) {
    const cia402::FbStatus status = PowerRobotAxis(
        robot.robot_axes[i], robot.cycle_data.robot_feedback[i], enable);
    if (status == cia402::FbStatus::kError) {
      has_error = true;
    }
  }

  return has_error ? RuntimeResult::kError : RuntimeResult::kSuccess;
}

// 每周期根据公共服务请求为所有机器人本体轴请求相同模式。
RuntimeResult SwitchRobotMode(RobotRuntimeData& robot) {
  if (robot.cycle_data.service.switch_mode == 0) {
    return RuntimeResult::kSuccess;
  }

  if (robot.cycle_data.robot_axis_count > robot_interface::kMaxRobotAxisCount) {
    return RuntimeResult::kError;
  }

  const robot_interface::RobotMode target_mode =
      robot.cycle_data.service.target_mode;
  bool has_error = false;
  for (uint8_t i = 0; i < robot.cycle_data.robot_axis_count; ++i) {
    const cia402::FbStatus status = SwitchRobotAxisMode(
        robot.robot_axes[i], robot.cycle_data.robot_feedback[i],
        static_cast<cia402::AxisMode>(target_mode));
    if (status == cia402::FbStatus::kError) {
      has_error = true;
    }
  }

  return has_error ? RuntimeResult::kError : RuntimeResult::kSuccess;
}

// 每周期根据公共服务请求处理回零状态，不负责回零前后的模式切换。
RuntimeResult HomeRobot(RobotRuntimeData& robot) {
  if (robot.cycle_data.robot_axis_count > robot_interface::kMaxRobotAxisCount) {
    return RuntimeResult::kError;
  }

  const bool start = robot.cycle_data.service.home != 0;
  bool has_error = false;
  for (uint8_t i = 0; i < robot.cycle_data.robot_axis_count; ++i) {
    const cia402::FbStatus status = HomeRobotAxis(
        robot.robot_axes[i], robot.cycle_data.robot_feedback[i], start,
        false);
    if (status == cia402::FbStatus::kError) {
      has_error = true;
    }
  }

  return has_error ? RuntimeResult::kError : RuntimeResult::kSuccess;
}

// 每周期根据公共服务请求处理清错或撤销 fault reset。
RuntimeResult ClearRobotError(RobotRuntimeData& robot) {
  if (robot.cycle_data.robot_axis_count > robot_interface::kMaxRobotAxisCount) {
    return RuntimeResult::kError;
  }

  const bool execute = robot.cycle_data.service.clear_error != 0;
  bool has_error = false;
  for (uint8_t i = 0; i < robot.cycle_data.robot_axis_count; ++i) {
    const cia402::FbStatus status = ClearRobotAxisError(
        robot.robot_axes[i], robot.cycle_data.robot_feedback[i], execute);
    if (status == cia402::FbStatus::kError) {
      has_error = true;
    }
  }

  return has_error ? RuntimeResult::kError : RuntimeResult::kSuccess;
}

}  // namespace robot_runtime
