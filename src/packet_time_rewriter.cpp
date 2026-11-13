#include "joint_sensor_player/packet_time_rewriter.hpp"

#include <algorithm>
#include <limits>
#include <optional>
#include <stdexcept>

namespace joint_sensor_player {
namespace {

struct Layout {
  LidarPacketProtocol protocol{};
  std::size_t timestamp_offset{};
  std::size_t utc_offset{};
  std::size_t sequence_offset{};
  std::size_t crc_offset{};
  std::size_t crc_begin{};
  bool has_crc{};
};

std::uint32_t read_le32(const std::vector<std::uint8_t>& packet,
                        std::size_t offset) {
  return static_cast<std::uint32_t>(packet[offset]) |
         (static_cast<std::uint32_t>(packet[offset + 1]) << 8U) |
         (static_cast<std::uint32_t>(packet[offset + 2]) << 16U) |
         (static_cast<std::uint32_t>(packet[offset + 3]) << 24U);
}

void write_le32(std::vector<std::uint8_t>& packet, std::size_t offset,
                std::uint32_t value) {
  packet[offset] = static_cast<std::uint8_t>(value);
  packet[offset + 1] = static_cast<std::uint8_t>(value >> 8U);
  packet[offset + 2] = static_cast<std::uint8_t>(value >> 16U);
  packet[offset + 3] = static_cast<std::uint8_t>(value >> 24U);
}

std::uint32_t read_utc_seconds(const std::vector<std::uint8_t>& packet,
                               std::size_t offset) {
  if (packet[offset] != 0 || packet[offset + 1] != 0) {
    throw std::runtime_error("暂不支持日历格式雷达UTC字段");
  }
  return (static_cast<std::uint32_t>(packet[offset + 2]) << 24U) |
         (static_cast<std::uint32_t>(packet[offset + 3]) << 16U) |
         (static_cast<std::uint32_t>(packet[offset + 4]) << 8U) |
         static_cast<std::uint32_t>(packet[offset + 5]);
}

void write_utc_seconds(std::vector<std::uint8_t>& packet, std::size_t offset,
                       std::uint32_t seconds) {
  packet[offset] = 0;
  packet[offset + 1] = 0;
  packet[offset + 2] = static_cast<std::uint8_t>(seconds >> 24U);
  packet[offset + 3] = static_cast<std::uint8_t>(seconds >> 16U);
  packet[offset + 4] = static_cast<std::uint8_t>(seconds >> 8U);
  packet[offset + 5] = static_cast<std::uint8_t>(seconds);
}

Layout locate(const std::vector<std::uint8_t>& packet) {
  if (packet.size() >= 12 && packet[0] == 0xEE && packet[1] == 0xFF &&
      packet[2] == 0x04 && packet[3] == 0x03) {
    const auto lasers = static_cast<std::size_t>(packet[6]);
    const auto blocks = static_cast<std::size_t>(packet[7]);
    if (lasers == 0 || blocks == 0 || (packet[11] & 0x01U) == 0) {
      throw std::runtime_error("UDP4.3头部或序号标志非法");
    }
    const auto body = (2U + 1U + 4U * lasers) * blocks;
    const auto sequence = 6U + 6U + body + 4U + 32U;
    if (sequence < 32 || sequence + 8 > packet.size()) {
      throw std::runtime_error("UDP4.3尾部布局越界");
    }
    return {LidarPacketProtocol::MainUdp43, sequence - 12U,
            sequence - 6U, sequence, sequence + 4U, sequence - 32U, true};
  }
  if (packet.size() == 554 && packet[0] == 0xEE && packet[1] == 0xFF &&
      packet[2] == 0x07 && packet[3] == 0x03) {
    return {LidarPacketProtocol::ShortUdp73, 525, 519, 530, 0, 0, false};
  }
  throw std::runtime_error("无法识别雷达UDP4.3/7.3协议");
}

LidarPacketInfo inspect_with_layout(const std::vector<std::uint8_t>& packet,
                                    const Layout& layout) {
  if (layout.has_crc) {
    const auto actual = read_le32(packet, layout.crc_offset);
    const auto expected =
        lidar_crc32(packet.data() + layout.crc_begin,
                    layout.crc_offset - layout.crc_begin);
    if (actual != expected) throw std::runtime_error("UDP4.3尾部CRC错误");
  }
  const auto seconds = read_utc_seconds(packet, layout.utc_offset);
  const auto micros = read_le32(packet, layout.timestamp_offset);
  if (micros >= 1000000U) throw std::runtime_error("雷达微秒时间戳非法");
  const auto time_us =
      static_cast<std::uint64_t>(seconds) * 1000000ULL + micros;
  if (time_us == 0) throw std::runtime_error("雷达包内时间为零");
  return {layout.protocol, time_us,
          read_le32(packet, layout.sequence_offset)};
}

}  // namespace

std::uint32_t lidar_crc32(const std::uint8_t* data, std::size_t size) {
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1U) ^
            (0xEDB88320U & (0U - static_cast<std::uint32_t>(crc & 1U)));
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

LidarPacketInfo inspect_lidar_packet(
    const std::vector<std::uint8_t>& packet) {
  const auto layout = locate(packet);
  return inspect_with_layout(packet, layout);
}

void rewrite_lidar_packet(std::vector<std::uint8_t>& packet,
                          std::uint64_t target_time_us,
                          std::uint32_t target_sequence) {
  const auto layout = locate(packet);
  (void)inspect_with_layout(packet, layout);
  const auto seconds = target_time_us / 1000000ULL;
  if (seconds > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("目标雷达UTC秒超过32位协议范围");
  }
  write_le32(packet, layout.timestamp_offset,
             static_cast<std::uint32_t>(target_time_us % 1000000ULL));
  write_utc_seconds(packet, layout.utc_offset,
                    static_cast<std::uint32_t>(seconds));
  write_le32(packet, layout.sequence_offset, target_sequence);
  if (layout.has_crc) {
    write_le32(packet, layout.crc_offset,
               lidar_crc32(packet.data() + layout.crc_begin,
                           layout.crc_offset - layout.crc_begin));
  }
}

LidarPacketRewriter LidarPacketRewriter::build(
    const McapCatalog& catalog,
    const std::vector<LidarChannelConfig>& channels,
    const PlaybackWindow& window, std::uint64_t cycle_period_ns,
    std::uint64_t anchor_utc_us) {
  return build_impl(
      catalog, channels, window, cycle_period_ns, anchor_utc_us);
}

LidarPacketRewriter LidarPacketRewriter::build_single_pass(
    const McapCatalog& catalog,
    const std::vector<LidarChannelConfig>& channels,
    const PlaybackWindow& window, std::uint64_t anchor_utc_us) {
  return build_impl(catalog, channels, window, std::nullopt, anchor_utc_us);
}

LidarPacketRewriter LidarPacketRewriter::build_impl(
    const McapCatalog& catalog,
    const std::vector<LidarChannelConfig>& channels,
    const PlaybackWindow& window,
    std::optional<std::uint64_t> cycle_period_ns,
    std::uint64_t anchor_utc_us) {
  if (cycle_period_ns &&
      (*cycle_period_ns == 0 || *cycle_period_ns % 1000ULL != 0)) {
    throw std::runtime_error("真雷达循环周期必须是非零整微秒");
  }
  LidarPacketRewriter result;
  if (cycle_period_ns) {
    result.cycle_period_us_ = *cycle_period_ns / 1000ULL;
  }
  result.anchor_utc_us_ = anchor_utc_us;
  std::optional<std::uint64_t> source_origin;
  std::optional<std::uint64_t> source_last;

  for (const auto& channel : channels) {
    if (!channel.enabled) continue;
    const auto found = catalog.frames.find(channel.topic);
    if (found == catalog.frames.end()) {
      throw std::runtime_error("真雷达时间预检缺少topic: " + channel.topic);
    }
    std::optional<std::uint64_t> previous_time;
    std::optional<LidarPacketProtocol> expected_protocol;
    std::optional<std::uint64_t> channel_first;
    for (const auto& frame : found->second) {
      if (frame.source_timestamp_ns < window.source_start_ns ||
          frame.source_timestamp_ns > window.source_end_ns) {
        continue;
      }
      for (const auto& packet : frame.packets) {
        const auto info = inspect_lidar_packet(packet);
        const auto channel_protocol =
            channel.packet_size == 1118
                ? LidarPacketProtocol::MainUdp43
                : LidarPacketProtocol::ShortUdp73;
        if (info.protocol != channel_protocol) {
          throw std::runtime_error("雷达topic协议与槽位配置不匹配: " +
                                   channel.topic);
        }
        if (expected_protocol && *expected_protocol != info.protocol) {
          throw std::runtime_error("同一雷达topic混入不同协议: " +
                                   channel.topic);
        }
        if (previous_time && info.sensor_time_us <= *previous_time) {
          throw std::runtime_error("雷达包内时间未严格递增: " +
                                   channel.topic);
        }
        expected_protocol = info.protocol;
        if (!channel_first) channel_first = info.sensor_time_us;
        previous_time = info.sensor_time_us;
        source_origin = source_origin
                            ? std::min(*source_origin, info.sensor_time_us)
                            : info.sensor_time_us;
        source_last = source_last
                          ? std::max(*source_last, info.sensor_time_us)
                          : info.sensor_time_us;
      }
    }
    if (!expected_protocol) {
      throw std::runtime_error("播放窗口内没有可重写雷达包: " +
                               channel.topic);
    }
    result.protocols_.emplace(channel.topic, *expected_protocol);
    result.channel_ranges_.emplace(
        channel.topic, ChannelTimeRange{*channel_first, *previous_time});
  }
  if (!source_origin || !source_last) {
    throw std::runtime_error("没有可建立真雷达时间轴的数据");
  }
  if (cycle_period_ns &&
      *source_last - *source_origin >= result.cycle_period_us_) {
    throw std::runtime_error("循环周期不足以保证包内时间跨轮递增");
  }
  result.source_origin_us_ = *source_origin;
  result.source_last_us_ = *source_last;
  result.sequence_plans_ = build_sequence_plans(
      catalog, channels, SequenceMode::Continuous, window.source_start_ns,
      window.source_end_ns);
  return result;
}

std::vector<std::uint8_t> LidarPacketRewriter::rewrite(
    const std::string& topic, const std::vector<std::uint8_t>& packet,
    std::uint64_t cycle) const {
  const auto protocol = protocols_.find(topic);
  const auto sequence_plan = sequence_plans_.find(topic);
  if (protocol == protocols_.end() || sequence_plan == sequence_plans_.end()) {
    throw std::runtime_error("缺少真雷达topic重写计划: " + topic);
  }
  const auto info = inspect_lidar_packet(packet);
  if (info.protocol != protocol->second ||
      info.sensor_time_us < source_origin_us_) {
    throw std::runtime_error("雷达包不符合预检重写计划: " + topic);
  }
  if (cycle != 0 && cycle_period_us_ == 0) {
    throw std::runtime_error("单轮雷达重写计划不支持非零cycle");
  }
  if (cycle != 0 &&
      cycle >
          (std::numeric_limits<std::uint64_t>::max() -
           anchor_utc_us_ - (info.sensor_time_us - source_origin_us_)) /
              cycle_period_us_) {
    throw std::runtime_error("真雷达目标时间溢出");
  }
  const auto target_time =
      anchor_utc_us_ + (info.sensor_time_us - source_origin_us_) +
      cycle * cycle_period_us_;
  const auto sequence_delta =
      static_cast<std::uint32_t>(cycle *
                                 sequence_plan->second.cycle_span);
  auto output = packet;
  rewrite_lidar_packet(output, target_time, info.sequence + sequence_delta);
  return output;
}

}  // namespace joint_sensor_player
