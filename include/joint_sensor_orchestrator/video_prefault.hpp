#pragma once

// 10 秒整窗路径用。把共同窗口里的 UYVY 拷进 /dev/shm 的稀疏文件，
// 让后面的 pread 打在内存盘上。40 秒滑动窗口不走这里。
// 视频窗口不 mlock：锁定额度优先留给点云。

#include "joint_sensor_orchestrator/unified_timeline.hpp"
#include "joint_sensor_orchestrator/video_cache.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <utility>
#include <vector>

namespace joint_sensor_orchestrator {

struct VideoPrefaultOptions {
  std::uint64_t max_lock_bytes{0};
  bool drop_outside_pages{false};
  // 把播放窗口里的帧拷进 stage_dir 下的稀疏文件，默认在 /dev/shm。
  // 这样后面的 pread 打在内存盘上。40 秒滑动窗口不打开这个开关。
  bool stage_to_shm{false};
  std::filesystem::path stage_dir{};
};

// 握着映射，以及可选的 /dev/shm 暂存文件。对象析构时放开。
class VideoPlaybackPin {
 public:
  VideoPlaybackPin() = default;
  VideoPlaybackPin(const VideoPlaybackPin&) = delete;
  VideoPlaybackPin& operator=(const VideoPlaybackPin&) = delete;
  VideoPlaybackPin(VideoPlaybackPin&& other) noexcept;
  VideoPlaybackPin& operator=(VideoPlaybackPin&& other) noexcept;
  ~VideoPlaybackPin();

  std::uint64_t frames_prefaulted() const noexcept {
    return frames_prefaulted_;
  }
  std::uint64_t outside_bytes_dontneed() const noexcept {
    return outside_bytes_dontneed_;
  }
  std::uint64_t locked_bytes() const noexcept { return locked_bytes_; }
  std::uint64_t staged_bytes() const noexcept { return staged_bytes_; }
  bool has_staged_payloads() const noexcept { return staged_bytes_ > 0; }

  void apply_staged_payload_paths(VideoCache& cache) const;

 private:
  friend VideoPlaybackPin prefault_and_pin_video_playback_window(
      const VideoCache& cache, SourceWindow window,
      std::optional<std::uint64_t> max_relative_ns,
      const VideoPrefaultOptions& options);

  struct Mapping {
    void* address{};
    std::size_t length{};
  };

  void release() noexcept;

  std::vector<Mapping> mappings_;
  std::array<std::filesystem::path, 3> staged_payload_paths_{};
  std::filesystem::path staged_root_;
  std::uint64_t frames_prefaulted_{};
  std::uint64_t outside_bytes_dontneed_{};
  std::uint64_t locked_bytes_{};
  std::uint64_t staged_bytes_{};
};

VideoPlaybackPin prefault_and_pin_video_playback_window(
    const VideoCache& cache, SourceWindow window,
    std::optional<std::uint64_t> max_relative_ns,
    const VideoPrefaultOptions& options);

VideoPlaybackPin prefault_and_pin_video_playback_window(
    const VideoCache& cache, SourceWindow window,
    std::optional<std::uint64_t> max_relative_ns = std::nullopt,
    std::uint64_t max_lock_bytes = 0, bool drop_outside_pages = false);

std::uint64_t prefault_video_playback_window(
    const VideoCache& cache, SourceWindow window,
    std::optional<std::uint64_t> max_relative_ns = std::nullopt);

}  // namespace joint_sensor_orchestrator
