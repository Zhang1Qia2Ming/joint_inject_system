#include "joint_sensor_player/clock_health.hpp"

#include <time.h>

#include <filesystem>
#include <stdexcept>

namespace joint_sensor_player {

ClockHealth validate_clock_health(std::uint64_t realtime_us,
                                  bool synchronized,
                                  bool require_synchronized) {
  constexpr std::uint64_t kUnix2020Us = 1577836800000000ULL;
  constexpr std::uint64_t kMaxProtocolUs =
      0xFFFFFFFFULL * 1000000ULL + 999999ULL;
  if (realtime_us < kUnix2020Us || realtime_us > kMaxProtocolUs) {
    throw std::runtime_error("系统UTC不在雷达协议可用时间范围");
  }
  if (require_synchronized && !synchronized) {
    throw std::runtime_error(
        "系统时钟未同步；仅离线压测可设置clock.require_synchronized=false");
  }
  return {realtime_us, synchronized, !synchronized};
}

ClockHealth inspect_system_clock(bool require_synchronized) {
  timespec realtime{};
  if (::clock_gettime(CLOCK_REALTIME, &realtime) != 0) {
    throw std::runtime_error("clock_gettime(CLOCK_REALTIME)失败");
  }
  const auto realtime_us =
      static_cast<std::uint64_t>(realtime.tv_sec) * 1000000ULL +
      static_cast<std::uint64_t>(realtime.tv_nsec) / 1000ULL;
  const bool synchronized =
      std::filesystem::exists("/run/systemd/timesync/synchronized");
  return validate_clock_health(realtime_us, synchronized,
                               require_synchronized);
}

}  // namespace joint_sensor_player
