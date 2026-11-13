#pragma once

#include "joint_sensor_orchestrator/frame_buffer_pool.hpp"
#include "joint_sensor_orchestrator/video_cache.hpp"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <vector>

namespace joint_sensor_orchestrator {

using AheadPread = std::function<std::ptrdiff_t(int, void*, std::size_t,
                                               std::uint64_t)>;

// Sliding RAM window of unique source frames. Disk reads stay in the filler;
// playback threads only wait for a frame that is already resident.
class VideoAheadCache {
 public:
  VideoAheadCache(const VideoCache& cache, std::uint64_t prefetch_ns,
                  AheadPread pread);
  ~VideoAheadCache();

  VideoAheadCache(const VideoAheadCache&) = delete;
  VideoAheadCache& operator=(const VideoAheadCache&) = delete;

  // Read every not-yet-resident source frame in [origin, origin + prefetch).
  void prefetch_from(std::size_t lane, std::uint64_t origin_ns,
                     const std::function<bool()>& should_stop);
  // Read at most one missing frame in that window. Returns false when the
  // window is already resident or the lane has nothing left to read.
  // end_index is exclusive. Frames at or past it are not played and are
  // not read, even when their timestamps still fall inside the window.
  bool prefetch_one(
      std::size_t lane, std::uint64_t origin_ns,
      const std::function<bool()>& should_stop,
      std::uint64_t end_index = std::numeric_limits<std::uint64_t>::max());

  VideoFrameBufferLease acquire(std::size_t lane, std::uint64_t source_index,
                                const std::function<bool()>& should_stop);
  void release_before(std::size_t lane, std::uint64_t source_index);
  std::uint64_t next_index(std::size_t lane) const;
  // Timestamp of the first frame in the prefetch window that is not in RAM.
  // Empty when that window is already full or the lane is exhausted.
  std::optional<std::uint64_t> next_missing_ns(
      std::size_t lane,
      std::uint64_t end_index = std::numeric_limits<std::uint64_t>::max())
      const;
  std::size_t resident_count(std::size_t lane) const;
  bool was_read(std::size_t lane, std::uint64_t source_index) const;
  void request_stop() noexcept;
  bool stopped() const noexcept { return stopped_.load(); }

 private:
  struct LaneState {
    int fd{-1};
    std::uint64_t released_before{};
    std::map<std::uint64_t, VideoFrameBufferLease> resident;
    std::set<std::uint64_t> read;
  };

  bool prefetch_one_locked(std::unique_lock<std::mutex>& lock,
                           std::size_t lane, std::uint64_t origin_ns,
                           const std::function<bool()>& should_stop,
                           std::uint64_t end_index);
  VideoFrameBufferLease take_buffer(std::size_t frame_bytes);
  void recycle(VideoFrameBuffer* raw);

  const VideoCache& cache_;
  std::uint64_t prefetch_ns_;
  AheadPread pread_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::atomic<bool> stopped_{false};
  bool direct_{false};
  void* scratch_{nullptr};
  std::size_t scratch_bytes_{0};
  std::vector<VideoFrameBufferLease> free_;
  std::array<LaneState, 3> lanes_{};
};

}  // namespace joint_sensor_orchestrator
