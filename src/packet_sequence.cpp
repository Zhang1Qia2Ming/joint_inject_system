#include "joint_sensor_player/packet_sequence.hpp"

#include <optional>
#include <stdexcept>

namespace joint_sensor_player {
namespace {

struct PacketSequenceLocation {
  PacketProtocol protocol;
  std::size_t offset;
};

std::optional<PacketSequenceLocation> locate_sequence(
    const std::vector<std::uint8_t>& packet) {
  if (packet.size() >= 12 && packet[0] == 0xEE && packet[1] == 0xFF &&
      packet[2] == 0x04 && packet[3] == 0x03) {
    const auto lasers = static_cast<std::size_t>(packet[6]);
    const auto blocks = static_cast<std::size_t>(packet[7]);
    if (lasers == 0 || blocks == 0 || (packet[11] & 0x01U) == 0) {
      return std::nullopt;
    }
    const auto body = (2U + 1U + 4U * lasers) * blocks;
    const auto offset = 6U + 6U + body + 4U + 32U;
    if (offset + sizeof(std::uint32_t) > packet.size()) return std::nullopt;
    return PacketSequenceLocation{PacketProtocol::MainUdp43, offset};
  }
  if (packet.size() == 554 && packet[0] == 0xEE && packet[1] == 0xFF &&
      packet[2] == 0x07 && packet[3] == 0x03) {
    constexpr std::size_t kShortSequenceOffset = 530;
    return PacketSequenceLocation{PacketProtocol::ShortUdp73,
                                  kShortSequenceOffset};
  }
  return std::nullopt;
}

std::uint32_t read_sequence(const std::vector<std::uint8_t>& packet,
                            std::size_t offset) {
  return static_cast<std::uint32_t>(packet[offset]) |
         (static_cast<std::uint32_t>(packet[offset + 1]) << 8U) |
         (static_cast<std::uint32_t>(packet[offset + 2]) << 16U) |
         (static_cast<std::uint32_t>(packet[offset + 3]) << 24U);
}

void write_sequence(std::vector<std::uint8_t>& packet, std::size_t offset,
                    std::uint32_t value) {
  packet[offset] = static_cast<std::uint8_t>(value & 0xFFU);
  packet[offset + 1] =
      static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  packet[offset + 2] =
      static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  packet[offset + 3] =
      static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

}  // namespace

SequencePlans build_sequence_plans(
    const McapCatalog& catalog,
    const std::vector<LidarChannelConfig>& channels,
    SequenceMode mode, std::uint64_t source_start_ns,
    std::uint64_t source_end_ns) {
  SequencePlans plans;
  if (mode == SequenceMode::Preserve) return plans;

  for (const auto& channel : channels) {
    if (!channel.enabled) continue;
    const auto frames_it = catalog.frames.find(channel.topic);
    if (frames_it == catalog.frames.end() || frames_it->second.empty()) continue;

    std::optional<PacketSequenceLocation> expected;
    std::optional<std::uint32_t> first;
    std::optional<std::uint32_t> previous;
    for (const auto& frame : frames_it->second) {
      if (frame.source_timestamp_ns < source_start_ns ||
          frame.source_timestamp_ns > source_end_ns) {
        continue;
      }
      for (const auto& packet : frame.packets) {
        const auto location = locate_sequence(packet);
        if (!location) {
          throw std::runtime_error("无法识别雷达UDP序号格式: " +
                                   channel.topic);
        }
        if (!expected) {
          expected = location;
        } else if (expected->protocol != location->protocol ||
                   expected->offset != location->offset) {
          throw std::runtime_error("同一topic混入不同UDP序号格式: " +
                                   channel.topic);
        }
        const auto sequence = read_sequence(packet, location->offset);
        if (!first) first = sequence;
        if (previous) {
          const std::uint32_t delta = sequence - *previous;
          if (delta == 0 || delta >= 0x80000000U) {
            throw std::runtime_error("雷达UDP序号倒退或异常跳变: " +
                                     channel.topic);
          }
        }
        previous = sequence;
      }
    }
    if (!expected || !first || !previous) {
      throw std::runtime_error("雷达topic没有可校验UDP包: " + channel.topic);
    }
    const std::uint32_t span = *previous - *first + 1U;
    if (span == 0) {
      throw std::runtime_error("单轮雷达UDP序号跨度无效: " + channel.topic);
    }
    plans.emplace(channel.topic,
                  ChannelSequencePlan{expected->protocol, expected->offset,
                                      span});
  }
  return plans;
}

void advance_packet_sequence(std::vector<std::uint8_t>& packet,
                             const ChannelSequencePlan& plan) {
  const auto location = locate_sequence(packet);
  if (!location || location->protocol != plan.protocol ||
      location->offset != plan.sequence_offset) {
    throw std::runtime_error("循环发送时雷达UDP序号格式变化");
  }
  const auto current = read_sequence(packet, plan.sequence_offset);
  write_sequence(packet, plan.sequence_offset, current + plan.cycle_span);
}

}  // namespace joint_sensor_player
