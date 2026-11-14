#pragma once

// 三路视频的滑动内存窗口。读盘只发生在 filler 里。
// 播放线程只等「这一帧已经在内存里」，不再自己 pread。
// 窗口前面已经播过的帧会放回缓冲池，后面的帧按 prefetch_ns 继续补。

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

class VideoAheadCache {
 public:
  VideoAheadCache(const VideoCache& cache, std::uint64_t prefetch_ns,
                  AheadPread pread);
  ~VideoAheadCache();

  VideoAheadCache(const VideoAheadCache&) = delete;
  VideoAheadCache& operator=(const VideoAheadCache&) = delete;

  // 把 [origin, origin + prefetch) 里还没进内存的源帧读完。
  void prefetch_from(std::size_t lane, std::uint64_t origin_ns,
                     const std::function<bool()>& should_stop);
  // 这个窗口里最多再读一帧。窗口已满或这一路没有剩余帧时返回 false。
  // end_index 不含自身。到了或超过它的帧即使时间还在窗口里也不读。
  bool prefetch_one(
      std::size_t lane, std::uint64_t origin_ns,
      const std::function<bool()>& should_stop,
      std::uint64_t end_index = std::numeric_limits<std::uint64_t>::max());

  VideoFrameBufferLease acquire(std::size_t lane, std::uint64_t source_index,
                                const std::function<bool()>& should_stop);
  void release_before(std::size_t lane, std::uint64_t source_index);
  std::uint64_t next_index(std::size_t lane) const;
  // 预读窗口里第一帧还没进内存的源时间。窗口已满或这一路读完时为空。
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
