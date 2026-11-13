#pragma once

#include "joint_sensor_player/config.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace joint_sensor_player {

struct LidarFrame {
  std::uint64_t source_timestamp_ns{};
  std::uint64_t bag_log_time_ns{};
  std::uint32_t sequence{};
  std::filesystem::path source_file;
  std::vector<std::vector<std::uint8_t>> packets;
};

struct SegmentAnnotation {
  std::string topic;
  std::string kind;
  std::uint64_t timestamp_ns{};
  std::string detail;
};

struct McapCatalog {
  std::vector<std::filesystem::path> files;
  std::map<std::string, std::vector<LidarFrame>> frames;
  std::vector<SegmentAnnotation> annotations;
  std::size_t duplicate_messages{};
};

struct LidarFrameMetadata {
  std::uint64_t source_timestamp_ns{};
  std::uint64_t bag_log_time_ns{};
  std::uint32_t sequence{};
  std::uint32_t packet_count{};
  std::uint64_t first_packet_time_us{};
  std::uint64_t last_packet_time_us{};
  std::uint32_t first_packet_sequence{};
  std::uint32_t last_packet_sequence{};
  std::uint8_t protocol{};
  std::uint64_t payload_bytes{};
  std::array<std::uint8_t, 32> payload_sha256{};
};

struct LightweightMcapCatalog {
  std::vector<std::filesystem::path> files;
  std::map<std::string, std::vector<LidarFrameMetadata>> frames;
  std::vector<SegmentAnnotation> annotations;
  std::size_t duplicate_messages{};
  std::size_t retained_payload_bytes{};
  std::size_t peak_chunk_bytes{};
  std::uint64_t max_observed_disorder_ns{};
  std::size_t required_reorder_frames{};
};

struct McapStreamLimits {
  std::size_t max_reorder_frames{128};
};

struct McapStreamStats {
  std::size_t max_reorder_frames{};
  std::size_t frames_emitted{};
};

using McapFrameObserver =
    std::function<bool(const std::string&, LidarFrame)>;

McapCatalog scan_mcap_directory(
    const std::filesystem::path& directory,
    const std::vector<LidarChannelConfig>& channels);

LightweightMcapCatalog scan_mcap_directory_lightweight(
    const std::filesystem::path& directory,
    const std::vector<LidarChannelConfig>& channels);

McapStreamStats stream_mcap_frames(
    const LightweightMcapCatalog& catalog,
    const std::vector<LidarChannelConfig>& channels,
    McapStreamLimits limits, const McapFrameObserver& observer);

}  // namespace joint_sensor_player
