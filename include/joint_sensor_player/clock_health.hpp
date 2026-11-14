#pragma once

// 开播前看系统钟是否已同步。配置要求同步而当前未同步时，拒绝开播。

#include <cstdint>

namespace joint_sensor_player {

struct ClockHealth {
  std::uint64_t realtime_us{};
  bool synchronized{};
  bool degraded{};
};

ClockHealth validate_clock_health(std::uint64_t realtime_us,
                                  bool synchronized,
                                  bool require_synchronized);

ClockHealth inspect_system_clock(bool require_synchronized);

}  // namespace joint_sensor_player
