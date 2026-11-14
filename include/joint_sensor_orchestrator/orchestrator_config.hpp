#pragma once

// 一次联合播放的配置。lidar 沿用雷达播放器的配置；
// 这里额外决定视频走 Mock 还是 XDMA、三路车道、迟到门限和环视模拟清单。

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

// 一路真实视频。h2c 是这张卡上的 /dev/xdma*_h2c_N。
// source_fps 低于 output_fps 时，同一张源帧会在输出网格上重复。
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
  // 0 表示播完共同窗口里排好的槽就停，不用墙钟再截一刀。
  std::uint64_t play_duration_ms{0};
  // 0：把整个播放窗口搬进 /dev/shm。
  // >0：解码后的帧留在 SSD，内存里只留这么多毫秒的滑动窗口。
  std::uint32_t video_prefetch_ms{0};
  std::filesystem::path video_mcap{
      "/data/joint_sensor_player/data/video.mcap"};
  std::filesystem::path cache_dir{"/data/joint_sensor_player/cache"};
  // 每路写卡线程前面的队列深度。5 帧大约是 167 毫秒。
  std::uint32_t video_queue_frames{5};
  // 视频在写卡前采样。醒来比期限晚过这个值就停。
  std::uint32_t video_max_lateness_us{5000};
  // 雷达在 UDP 发送前采样，规则和视频相同。
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
