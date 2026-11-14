#pragma once

// 解码后的三路 UYVY 缓存。每路一个按帧号排列的裸文件，外加时间戳。
// 一帧是 3840x2160 的 UYVY，正好 16588800 字节，也是 4096 对齐的。
// 播放器只核对清单和文件，不在这里解码。

#include "joint_sensor_orchestrator/orchestrator_config.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace joint_sensor_orchestrator {

inline constexpr std::uint64_t kVideoFrameBytes = 3840ULL * 2160ULL * 2ULL;
inline constexpr char kVideoCacheConverterVersion[] =
    "joint-sensor-video-cache-1";

struct VideoCacheLane {
  std::string name;
  std::string topic;
  unsigned source_fps{};
  unsigned output_fps{};
  std::uint64_t frame_count{};
  std::uint64_t frame_bytes{};
  std::filesystem::path payload_path;
  std::filesystem::path timestamps_path;
  std::string sha256;
  std::uint64_t timestamps_byte_size{};
  std::string timestamps_sha256;
  std::vector<std::uint64_t> timestamps_ns;
};

struct VideoCache {
  unsigned version{};
  std::string dataset_id;
  std::string cache_key;
  std::string converter_version;
  std::filesystem::path source_mcap_path;
  std::uint64_t source_mcap_size{};
  std::uint64_t source_mcap_manifest_mtime_ns{};
  std::uint64_t source_mcap_current_mtime_ns{};
  bool source_metadata_changed{};
  std::array<VideoCacheLane, 3> lanes;
};

// 核对缓存目录和这次配置是否同一份数据。对不上就拒绝开播。
VideoCache load_video_cache(
    const std::filesystem::path& cache_dir,
    const OrchestratorConfig& expected_config,
    std::uint64_t expected_frame_bytes = kVideoFrameBytes);

}  // namespace joint_sensor_orchestrator
