#pragma once

#include "joint_sensor_player/config.hpp"
#include "joint_sensor_player/mcap.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace joint_sensor_player {

enum class PacketProtocol {
  MainUdp43,
  ShortUdp73,
};

struct ChannelSequencePlan {
  PacketProtocol protocol{};
  std::size_t sequence_offset{};
  std::uint32_t cycle_span{};
};

using SequencePlans =
    std::unordered_map<std::string, ChannelSequencePlan>;

SequencePlans build_sequence_plans(
    const McapCatalog& catalog,
    const std::vector<LidarChannelConfig>& channels,
    SequenceMode mode,
    std::uint64_t source_start_ns = 0,
    std::uint64_t source_end_ns = std::numeric_limits<std::uint64_t>::max());

void advance_packet_sequence(std::vector<std::uint8_t>& packet,
                             const ChannelSequencePlan& plan);

}  // namespace joint_sensor_player
