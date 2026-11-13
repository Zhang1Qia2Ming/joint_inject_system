#pragma once

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
  // Copy playback-window frames into sparse files under stage_dir
  // (default /dev/shm/joint_uyvy_stage) so XDMA reads RAM-backed tmpfs.
  bool stage_to_shm{false};
  std::filesystem::path stage_dir{};
};

// Holds mmap+mlock ranges and optional shm-staged payload files.
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
