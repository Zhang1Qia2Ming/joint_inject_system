#pragma once

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

namespace joint_sensor_orchestrator {

enum class PlaybackPhase {
  CheckingData,
  PreparingVideo,
  PreparingRadar,
  WaitingToSend,
  WaitingForEnter,
  Prefilling,
  Sending,
};

inline std::string format_seconds(std::chrono::milliseconds duration) {
  if (duration.count() < 0) duration = std::chrono::milliseconds::zero();
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << (static_cast<double>(duration.count()) / 1000.0) << 's';
  return out.str();
}

inline std::string format_local_time(
    std::chrono::system_clock::time_point time) {
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
      time.time_since_epoch());
  const auto seconds =
      std::chrono::duration_cast<std::chrono::seconds>(millis);
  const auto fraction = millis - seconds;
  const std::time_t calendar = std::chrono::system_clock::to_time_t(
      std::chrono::system_clock::time_point{seconds});
  std::tm local{};
  localtime_r(&calendar, &local);
  std::ostringstream out;
  out << std::put_time(&local, "%Y-%m-%d %H:%M:%S") << '.' << std::setw(3)
      << std::setfill('0') << fraction.count();
  return out.str();
}

inline std::string format_phase_status(
    PlaybackPhase phase, std::chrono::milliseconds primary = {},
    std::chrono::milliseconds secondary = {}) {
  switch (phase) {
    case PlaybackPhase::CheckingData:
      return "状态: 正在检查数据，尚未发送";
    case PlaybackPhase::PreparingVideo:
      return "状态: 正在准备视频窗口，尚未发送";
    case PlaybackPhase::PreparingRadar:
      return "状态: 正在准备雷达，尚未发送";
    case PlaybackPhase::WaitingToSend:
      return "状态: 等待开播，剩余 " + format_seconds(primary) + "，尚未发送";
    case PlaybackPhase::WaitingForEnter:
      return "状态: 倒计时结束，等待回车；按下后才预热队列，尚未发送";
    case PlaybackPhase::Prefilling:
      return "状态: 正在预填发送队列，剩余 " + format_seconds(primary) +
             "，尚未发送";
    case PlaybackPhase::Sending: {
      std::string line = "状态: 发送中 已发送 " + format_seconds(primary);
      if (secondary.count() > 0) {
        line += " / " + format_seconds(secondary);
      }
      return line;
    }
  }
  return "状态: 未知";
}

inline std::string format_send_start(
    std::chrono::system_clock::time_point time) {
  return "发送开始: " + format_local_time(time);
}

inline std::string format_send_end(
    std::chrono::system_clock::time_point start,
    std::chrono::system_clock::time_point end) {
  const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      end - start);
  return "发送结束: " + format_local_time(end) +
         " 发送时长: " + format_seconds(duration);
}

}  // namespace joint_sensor_orchestrator
