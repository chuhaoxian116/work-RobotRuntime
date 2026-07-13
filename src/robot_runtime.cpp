#include "robot_runtime/robot_runtime.h"

namespace {

bool IsAxisAvailable(const robot_runtime::AxisRuntimeData& axis) {
  return axis.configured != 0 && axis.communication_valid != 0;
}

cia402::FbStatus PowerAxis(robot_runtime::AxisRuntimeData& axis, bool enable) {
  if (!IsAxisAvailable(axis)) {
    return cia402::FbStatus::kError;
  }

  return cia402::PowerAxis(axis.cia402_data, enable);
}

cia402::FbStatus SwitchAxisMode(robot_runtime::AxisRuntimeData& axis,
                                cia402::AxisMode target_mode) {
  if (!IsAxisAvailable(axis)) {
    return cia402::FbStatus::kError;
  }

  return cia402::SwitchMode(axis.cia402_data, target_mode);
}

cia402::FbStatus HomeAxis(robot_runtime::AxisRuntimeData& axis, bool start,
                          bool require_operation_enabled) {
  if (!IsAxisAvailable(axis)) {
    return cia402::FbStatus::kError;
  }

  return cia402::Homing(axis.cia402_data, start, require_operation_enabled);
}

cia402::FbStatus ClearAxisError(robot_runtime::AxisRuntimeData& axis) {
  if (!IsAxisAvailable(axis)) {
    return cia402::FbStatus::kError;
  }

  return cia402::ClearAxisError(axis.cia402_data);
}

}  // namespace

namespace robot_runtime {

cia402::FbStatus PowerRobot(RobotRuntimeData& robot, bool enable) {
  if (robot.robot_axis_count > kMaxRobotAxisCount) {
    return cia402::FbStatus::kError;
  }

  bool has_error = false;
  bool has_busy = false;
  for (uint8_t i = 0; i < robot.robot_axis_count; ++i) {
    const cia402::FbStatus status = PowerAxis(robot.robot_axes[i], enable);
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

cia402::FbStatus SwitchRobotMode(RobotRuntimeData& robot, cia402::AxisMode target_mode) {
  if (robot.robot_axis_count > kMaxRobotAxisCount) {
    return cia402::FbStatus::kError;
  }

  bool has_error = false;
  bool has_busy = false;
  for (uint8_t i = 0; i < robot.robot_axis_count; ++i) {
    const cia402::FbStatus status = SwitchAxisMode(robot.robot_axes[i], target_mode);
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

cia402::FbStatus HomeRobot(RobotRuntimeData& robot, bool start, bool require_operation_enabled) {
  if (robot.robot_axis_count > kMaxRobotAxisCount) {
    return cia402::FbStatus::kError;
  }

  bool has_error = false;
  bool has_busy = false;
  for (uint8_t i = 0; i < robot.robot_axis_count; ++i) {
    const cia402::FbStatus status = HomeAxis(robot.robot_axes[i], start, require_operation_enabled);
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

cia402::FbStatus ClearRobotError(RobotRuntimeData& robot) {
  if (robot.robot_axis_count > kMaxRobotAxisCount) {
    return cia402::FbStatus::kError;
  }

  bool has_error = false;
  bool has_busy = false;
  for (uint8_t i = 0; i < robot.robot_axis_count; ++i) {
    const cia402::FbStatus status = ClearAxisError(robot.robot_axes[i]);
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
