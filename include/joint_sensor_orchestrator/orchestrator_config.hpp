#pragma once

#include "joint_sensor_player/config.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>

namespace joint_sensor_orchestrator {

enum class VideoBackend {
  Mock,
  Xdma,
};

struct VideoLaneConfig {
  std::string name;
  std::string topic;
  unsigned h2c{};
  unsigned source_fps{};
  unsigned output_fps{30};
};

struct OrchestratorConfig {
  joint_sensor_player::PlayerConfig lidar;
  VideoBackend video_backend{VideoBackend::Mock};
  bool strict{true};
  bool allow_video_only_smoke{false};
  bool wait_for_enter{false};
  std::uint64_t play_duration_ms{0};
  // 0: stage the whole playback window into /dev/shm.
  // >0: keep decoded frames on SSD and retain this many milliseconds in RAM.
  std::uint32_t video_prefetch_ms{0};
  std::filesystem::path video_mcap{
      "/data/joint_sensor_player/data/video.mcap"};
  std::filesystem::path cache_dir{"/data/joint_sensor_player/cache"};
  std::uint32_t video_queue_frames{5};
  std::uint32_t video_max_lateness_us{5000};
  std::uint32_t lidar_max_lateness_us{20000};
  std::uint64_t loop_count{1};
  std::array<VideoLaneConfig, 3> lanes{{
      {"tx1", "/camera/tx1/h265", 0, 10, 30},
      {"tx2", "/camera/tx2/h265", 1, 30, 30},
      {"tx4", "/camera/tx4/h265", 2, 10, 30},
  }};
  // Empty unless this run also feeds Sensor4-7 up to, but not into, H2C.
  std::filesystem::path shadow_manifest;
};

OrchestratorConfig load_orchestrator_config(
    const std::filesystem::path& path);

}  // namespace joint_sensor_orchestrator
