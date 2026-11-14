#pragma once

// 把雷达包里的传感器时间改到这次播放的 UTC 锚点上，并重算校验。
// 视频帧没有这一步。锚点来自 StartGate 放出的 utc_anchor_us。

#include "joint_sensor_player/config.hpp"
#include "joint_sensor_player/mcap.hpp"
#include "joint_sensor_player/packet_sequence.hpp"
#include "joint_sensor_player/timeline.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace joint_sensor_player {

enum class LidarPacketProtocol {
  MainUdp43,
  ShortUdp73,
};

struct LidarPacketInfo {
  LidarPacketProtocol protocol{};
  std::uint64_t sensor_time_us{};
  std::uint32_t sequence{};
};

struct ChannelTimeRange {
  std::uint64_t first_us{};
  std::uint64_t last_us{};
};

std::uint32_t lidar_crc32(const std::uint8_t* data, std::size_t size);

LidarPacketInfo inspect_lidar_packet(
    const std::vector<std::uint8_t>& packet);

void rewrite_lidar_packet(std::vector<std::uint8_t>& packet,
                          std::uint64_t target_time_us,
                          std::uint32_t target_sequence);

class LidarPacketRewriter {
 public:
  // 按共同窗口和 UTC 锚点建立每路的时间映射。发送时逐包调用 rewrite。
  static LidarPacketRewriter build_single_pass(
      const McapCatalog& catalog,
      const std::vector<LidarChannelConfig>& channels,
      const PlaybackWindow& window, std::uint64_t anchor_utc_us);

  static LidarPacketRewriter build(
      const McapCatalog& catalog,
      const std::vector<LidarChannelConfig>& channels,
      const PlaybackWindow& window, std::uint64_t cycle_period_ns,
      std::uint64_t anchor_utc_us);

  std::vector<std::uint8_t> rewrite(
      const std::string& topic, const std::vector<std::uint8_t>& packet,
      std::uint64_t cycle) const;

  std::uint64_t source_origin_us() const noexcept {
    return source_origin_us_;
  }
  std::uint64_t cycle_period_us() const noexcept {
    return cycle_period_us_;
  }
  std::uint64_t anchor_utc_us() const noexcept { return anchor_utc_us_; }
  const std::unordered_map<std::string, ChannelTimeRange>& channel_ranges()
      const noexcept {
    return channel_ranges_;
  }

 private:
  static LidarPacketRewriter build_impl(
      const McapCatalog& catalog,
      const std::vector<LidarChannelConfig>& channels,
      const PlaybackWindow& window,
      std::optional<std::uint64_t> cycle_period_ns,
      std::uint64_t anchor_utc_us);

  std::uint64_t source_origin_us_{};
  std::uint64_t source_last_us_{};
  std::uint64_t cycle_period_us_{};
  std::uint64_t anchor_utc_us_{};
  SequencePlans sequence_plans_;
  std::unordered_map<std::string, LidarPacketProtocol> protocols_;
  std::unordered_map<std::string, ChannelTimeRange> channel_ranges_;
};

}  // namespace joint_sensor_player
