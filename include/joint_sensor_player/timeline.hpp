#pragma once

#include "joint_sensor_player/config.hpp"
#include "joint_sensor_player/mcap.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace joint_sensor_player {

struct PlaybackWindow {
  std::uint64_t source_start_ns{};
  std::uint64_t source_end_ns{};
  std::size_t enabled_topics{};
};

PlaybackWindow build_playback_window(
    const McapCatalog& catalog,
    const std::vector<LidarChannelConfig>& channels,
    bool allow_partial_start);

std::uint64_t nominal_frame_period_ns(
    const McapCatalog& catalog,
    const std::vector<LidarChannelConfig>& channels);

std::uint64_t packet_deadline_ns(std::uint64_t play_start_ns,
                                 std::uint64_t source_start_ns,
                                 std::uint64_t frame_source_ns,
                                 std::size_t packet_index,
                                 std::uint64_t packet_interval_ns,
                                 std::int64_t time_offset_us);

}  // namespace joint_sensor_player
