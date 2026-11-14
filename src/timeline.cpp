// 从各路帧时间得到共同窗口，再把每个 UDP 包算到绝对发送期限。

#include "joint_sensor_player/timeline.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace joint_sensor_player {

PlaybackWindow build_playback_window(
    const McapCatalog& catalog,
    const std::vector<LidarChannelConfig>& channels,
    bool allow_partial_start) {
  PlaybackWindow window;
  std::uint64_t latest_start = 0;
  std::uint64_t earliest_end = std::numeric_limits<std::uint64_t>::max();
  for (const auto& channel : channels) {
    if (!channel.enabled) continue;
    const auto found = catalog.frames.find(channel.topic);
    if (found == catalog.frames.end() || found->second.empty()) {
      if (!allow_partial_start) {
        throw std::runtime_error("严格启动缺少topic: " + channel.topic);
      }
      continue;
    }
    ++window.enabled_topics;
    latest_start =
        std::max(latest_start, found->second.front().source_timestamp_ns);
    earliest_end =
        std::min(earliest_end, found->second.back().source_timestamp_ns);
  }
  if (window.enabled_topics == 0) {
    throw std::runtime_error("没有可播放的雷达topic");
  }
  if (latest_start > earliest_end) {
    throw std::runtime_error("雷达topic之间没有共同时间区间");
  }
  window.source_start_ns = latest_start;
  window.source_end_ns = earliest_end;
  return window;
}

std::uint64_t nominal_frame_period_ns(
    const McapCatalog& catalog,
    const std::vector<LidarChannelConfig>& channels) {
  std::vector<std::uint64_t> intervals;
  for (const auto& channel : channels) {
    if (!channel.enabled) continue;
    const auto found = catalog.frames.find(channel.topic);
    if (found == catalog.frames.end()) continue;
    const auto& frames = found->second;
    for (std::size_t index = 1; index < frames.size(); ++index) {
      const auto previous = frames[index - 1].source_timestamp_ns;
      const auto current = frames[index].source_timestamp_ns;
      if (current > previous) intervals.push_back(current - previous);
    }
  }
  if (intervals.empty()) {
    throw std::runtime_error("循环播放无法计算帧周期");
  }
  std::sort(intervals.begin(), intervals.end());
  const auto median = intervals[(intervals.size() - 1) / 2];
  if (median < 80000000ULL || median > 120000000ULL) {
    throw std::runtime_error("循环播放要求雷达帧率约为10Hz");
  }
  return median;
}

std::uint64_t packet_deadline_ns(std::uint64_t play_start_ns,
                                 std::uint64_t source_start_ns,
                                 std::uint64_t frame_source_ns,
                                 std::size_t packet_index,
                                 std::uint64_t packet_interval_ns,
                                 std::int64_t time_offset_us) {
  if (frame_source_ns < source_start_ns) {
    throw std::runtime_error("帧时间早于播放窗口");
  }
  if (packet_interval_ns != 0 &&
      packet_index >
          std::numeric_limits<std::uint64_t>::max() / packet_interval_ns) {
    throw std::runtime_error("发送期限溢出");
  }
  const auto relative = frame_source_ns - source_start_ns;
  const auto packet_offset =
      static_cast<std::uint64_t>(packet_index) * packet_interval_ns;
  if (play_start_ns > std::numeric_limits<std::uint64_t>::max() - relative ||
      play_start_ns + relative >
          std::numeric_limits<std::uint64_t>::max() - packet_offset) {
    throw std::runtime_error("发送期限溢出");
  }
  auto deadline = play_start_ns + relative + packet_offset;
  if (time_offset_us >= 0) {
    const auto offset = static_cast<std::uint64_t>(time_offset_us) * 1000;
    if (deadline > std::numeric_limits<std::uint64_t>::max() - offset) {
      throw std::runtime_error("发送期限溢出");
    }
    deadline += offset;
  } else {
    const auto offset =
        static_cast<std::uint64_t>(-(time_offset_us + 1)) * 1000 + 1000;
    if (deadline < offset) throw std::runtime_error("发送期限溢出");
    deadline -= offset;
  }
  return deadline;
}

}  // namespace joint_sensor_player
