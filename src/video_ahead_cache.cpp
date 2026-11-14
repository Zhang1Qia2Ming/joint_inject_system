// 滑动窗口的读盘实现。一帧拆成 1 MiB 的 pread，块与块之间让出 CPU。
// 更大的块会把同一台机器上的雷达发送顶过 100 毫秒门限。

#include "joint_sensor_orchestrator/video_ahead_cache.hpp"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

namespace joint_sensor_orchestrator {
namespace {

// A 16 MiB read, and a 4 MiB read on its own core, have both pushed a
// radar wake past the 100 ms gate (lateness_us=100022 and 100059).
// 1 MiB plus a yield stays under that gate on this machine.
constexpr std::size_t kPreadChunkBytes = 1ULL * 1024 * 1024;

}  // namespace

VideoAheadCache::VideoAheadCache(const VideoCache& cache,
                                 std::uint64_t prefetch_ns, AheadPread pread)
    : cache_(cache), prefetch_ns_(prefetch_ns), pread_(std::move(pread)) {
  if (!pread_) {
    throw std::invalid_argument("视频预读需要pread");
  }
  if (prefetch_ns_ == 0) {
    throw std::invalid_argument("视频预读窗口必须大于0");
  }
  for (std::size_t index = 0; index < lanes_.size(); ++index) {
    const auto& lane = cache_.lanes[index];
    if (lane.frame_bytes == 0 ||
        lane.frame_bytes > std::numeric_limits<std::size_t>::max()) {
      throw std::runtime_error("视频预读帧尺寸无效: " + lane.name);
    }
    const int fd = ::open(lane.payload_path.c_str(),
                          O_RDONLY | O_CLOEXEC | O_DIRECT);
    if (fd < 0) {
      throw std::runtime_error("无法打开视频预读缓存: " +
                               lane.payload_path.string());
    }
    lanes_[index].fd = fd;
    direct_ = true;
    if (lane.frame_bytes > scratch_bytes_) scratch_bytes_ = lane.frame_bytes;
  }
  if (scratch_bytes_ > 0) {
    void* aligned = nullptr;
    if (::posix_memalign(&aligned, 4096, scratch_bytes_) != 0) {
      throw std::bad_alloc();
    }
    scratch_ = aligned;
  }
}

VideoFrameBufferLease VideoAheadCache::take_buffer(std::size_t frame_bytes) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!free_.empty()) {
      auto buffer = std::move(free_.back());
      free_.pop_back();
      if (buffer) {
        if (buffer->size() != frame_bytes) buffer->resize(frame_bytes);
        return buffer;
      }
    }
  }
  auto* raw = new VideoFrameBuffer(frame_bytes);
  return VideoFrameBufferLease(raw, [this](VideoFrameBuffer* pointer) {
    recycle(pointer);
  });
}

void VideoAheadCache::recycle(VideoFrameBuffer* raw) {
  std::unique_ptr<VideoFrameBuffer> kept(raw);
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopped_.load()) return;
  free_.push_back(VideoFrameBufferLease(kept.release(), [this](VideoFrameBuffer* pointer) {
    recycle(pointer);
  }));
}

VideoAheadCache::~VideoAheadCache() {
  request_stop();
  std::free(scratch_);
  scratch_ = nullptr;
  for (auto& lane : lanes_) {
    if (lane.fd >= 0) {
      ::close(lane.fd);
      lane.fd = -1;
    }
  }
}

void VideoAheadCache::request_stop() noexcept {
  stopped_.store(true);
  ready_.notify_all();
}

void VideoAheadCache::prefetch_from(std::size_t lane, std::uint64_t origin_ns,
                                    const std::function<bool()>& should_stop) {
  while (prefetch_one(lane, origin_ns, should_stop)) {
  }
}

bool VideoAheadCache::prefetch_one(std::size_t lane, std::uint64_t origin_ns,
                                   const std::function<bool()>& should_stop,
                                   std::uint64_t end_index) {
  std::unique_lock<std::mutex> lock(mutex_);
  return prefetch_one_locked(lock, lane, origin_ns, should_stop, end_index);
}

bool VideoAheadCache::prefetch_one_locked(
    std::unique_lock<std::mutex>& lock, std::size_t lane,
    std::uint64_t origin_ns, const std::function<bool()>& should_stop,
    std::uint64_t end_index) {
  if (lane >= lanes_.size()) {
    throw std::out_of_range("视频预读通道越界");
  }
  if (stopped_.load() || (should_stop && should_stop())) return false;
  const auto& cached = cache_.lanes[lane];
  auto& state = lanes_[lane];
  const auto window_end = origin_ns > std::numeric_limits<std::uint64_t>::max() -
                                      prefetch_ns_
                              ? std::numeric_limits<std::uint64_t>::max()
                              : origin_ns + prefetch_ns_;
  std::optional<std::uint64_t> needed;
  for (std::uint64_t index = state.released_before;
       index < cached.timestamps_ns.size() && index < end_index; ++index) {
    const auto timestamp = cached.timestamps_ns[index];
    if (timestamp < origin_ns) continue;
    if (timestamp >= window_end) break;
    if (!state.resident.count(index)) {
      needed = index;
      break;
    }
  }
  if (!needed) return false;

  const auto source_index = *needed;
  const auto frame_bytes = static_cast<std::size_t>(cached.frame_bytes);
  if (source_index > std::numeric_limits<std::uint64_t>::max() / cached.frame_bytes) {
    throw std::overflow_error("视频预读偏移溢出: " + cached.name);
  }
  const auto offset = source_index * cached.frame_bytes;
  const int fd = state.fd;
  lock.unlock();

  auto buffer = take_buffer(frame_bytes);
  if (buffer && !buffer->empty()) {
    ::madvise(buffer->data(), buffer->size(), MADV_NOHUGEPAGE);
  }
  const auto address = reinterpret_cast<std::uintptr_t>(buffer->data());
  const bool direct_frame =
      direct_ && frame_bytes % 4096 == 0 && offset % 4096 == 0 &&
      address % 4096 == 0;
  if (direct_frame) {
    // Yield between chunks so a direct read cannot hold the core for a
    // whole frame. The chunk size itself stays well below 16 MiB.
    timespec started{};
    timespec finished{};
    ::clock_gettime(CLOCK_MONOTONIC, &started);
    std::size_t got = 0;
    while (got < frame_bytes) {
      if (stopped_.load() || (should_stop && should_stop())) {
        ready_.notify_all();
        return false;
      }
      const auto chunk = std::min(kPreadChunkBytes, frame_bytes - got);
      const auto count =
          pread_(fd, buffer->data() + got, chunk, offset + got);
      if (count < 0 || static_cast<std::size_t>(count) != chunk) {
        throw std::runtime_error("视频预读未读完整帧: " + cached.name);
      }
      got += static_cast<std::size_t>(count);
      if (got < frame_bytes) std::this_thread::yield();
    }
    ::clock_gettime(CLOCK_MONOTONIC, &finished);
    const auto elapsed_ms =
        (finished.tv_sec - started.tv_sec) * 1000.0 +
        (finished.tv_nsec - started.tv_nsec) / 1e6;
    if (elapsed_ms > 30.0) {
      std::fprintf(stderr, "视频预读偏慢 %s source=%llu %.1fms\n",
                   cached.name.c_str(),
                   static_cast<unsigned long long>(source_index), elapsed_ms);
    }
  } else {
    std::size_t got = 0;
    while (got < frame_bytes) {
      if (stopped_.load() || (should_stop && should_stop())) {
        ready_.notify_all();
        return false;
      }
      const auto chunk = std::min(kPreadChunkBytes, frame_bytes - got);
      const auto count =
          pread_(fd, buffer->data() + got, chunk, offset + got);
      if (count < 0 || static_cast<std::size_t>(count) != chunk) {
        throw std::runtime_error("视频预读未读完整帧: " + cached.name);
      }
      got += static_cast<std::size_t>(count);
      if (got < frame_bytes) std::this_thread::yield();
    }
  }

  lock.lock();
  state.read.insert(source_index);
  VideoFrameBufferLease discarded;
  if (source_index >= state.released_before) {
    state.resident[source_index] = std::move(buffer);
  } else {
    discarded = std::move(buffer);
  }
  ready_.notify_all();
  lock.unlock();
  return true;
}

VideoFrameBufferLease VideoAheadCache::acquire(
    std::size_t lane, std::uint64_t source_index,
    const std::function<bool()>& should_stop) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (lane >= lanes_.size()) {
    throw std::out_of_range("视频预读通道越界");
  }
  while (!stopped_.load() && !(should_stop && should_stop())) {
    const auto found = lanes_[lane].resident.find(source_index);
    if (found != lanes_[lane].resident.end()) return found->second;
    ready_.wait_for(lock, std::chrono::milliseconds(20));
  }
  return nullptr;
}

void VideoAheadCache::release_before(std::size_t lane,
                                     std::uint64_t source_index) {
  std::vector<VideoFrameBufferLease> dropped;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (lane >= lanes_.size()) return;
    auto& state = lanes_[lane];
    if (source_index > state.released_before) {
      state.released_before = source_index;
    }
    for (auto it = state.resident.begin(); it != state.resident.end();) {
      if (it->first < state.released_before) {
        dropped.push_back(std::move(it->second));
        it = state.resident.erase(it);
      } else {
        ++it;
      }
    }
    ready_.notify_all();
  }
}

std::uint64_t VideoAheadCache::next_index(std::size_t lane) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (lane >= lanes_.size()) return 0;
  return lanes_[lane].released_before;
}

std::optional<std::uint64_t> VideoAheadCache::next_missing_ns(
    std::size_t lane, std::uint64_t end_index) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (lane >= lanes_.size()) return std::nullopt;
  const auto& cached = cache_.lanes[lane];
  const auto& state = lanes_[lane];
  if (state.released_before >= cached.timestamps_ns.size() ||
      state.released_before >= end_index) {
    return std::nullopt;
  }
  const auto origin = cached.timestamps_ns[state.released_before];
  const auto window_end =
      origin > std::numeric_limits<std::uint64_t>::max() - prefetch_ns_
          ? std::numeric_limits<std::uint64_t>::max()
          : origin + prefetch_ns_;
  for (std::uint64_t index = state.released_before;
       index < cached.timestamps_ns.size() && index < end_index; ++index) {
    const auto timestamp = cached.timestamps_ns[index];
    if (timestamp >= window_end) break;
    if (!state.resident.count(index)) return timestamp;
  }
  return std::nullopt;
}

std::size_t VideoAheadCache::resident_count(std::size_t lane) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (lane >= lanes_.size()) return 0;
  return lanes_[lane].resident.size();
}

bool VideoAheadCache::was_read(std::size_t lane,
                               std::uint64_t source_index) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (lane >= lanes_.size()) return false;
  return lanes_[lane].read.count(source_index) != 0;
}

}  // namespace joint_sensor_orchestrator
