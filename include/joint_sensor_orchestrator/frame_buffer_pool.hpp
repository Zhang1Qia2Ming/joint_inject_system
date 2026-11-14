#pragma once

// 视频帧缓冲池。缓冲按 4096 字节对齐，O_DIRECT 的 pread 才能直接写进来。
// 没有预读窗口的路径用这个池；有滑动窗口时，窗口自己也复用同样对齐的缓冲。

#include "joint_sensor_player/player.hpp"

#include <stdlib.h>

#include <cstddef>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <vector>

namespace joint_sensor_orchestrator {

// O_DIRECT 的目标地址必须 4096 对齐。默认分配器保证不了这一点。
template <typename T>
struct PageAlignedAllocator {
  using value_type = T;
  PageAlignedAllocator() noexcept = default;
  template <typename U>
  explicit PageAlignedAllocator(const PageAlignedAllocator<U>&) noexcept {}

  T* allocate(std::size_t count) {
    if (count == 0) count = 1;
    if (count > std::numeric_limits<std::size_t>::max() / sizeof(T)) {
      throw std::bad_alloc();
    }
    void* memory = nullptr;
    if (::posix_memalign(&memory, 4096, count * sizeof(T)) != 0) {
      throw std::bad_alloc();
    }
    return static_cast<T*>(memory);
  }

  void deallocate(T* pointer, std::size_t) noexcept { std::free(pointer); }

  template <typename U>
  bool operator==(const PageAlignedAllocator<U>&) const noexcept {
    return true;
  }
  template <typename U>
  bool operator!=(const PageAlignedAllocator<U>&) const noexcept {
    return false;
  }
};

using VideoFrameBuffer =
    std::vector<std::byte, PageAlignedAllocator<std::byte>>;
using VideoFrameBufferLease = std::shared_ptr<VideoFrameBuffer>;

class FrameBufferPool {
 public:
  FrameBufferPool(std::size_t capacity, std::size_t frame_bytes);
  ~FrameBufferPool();
  FrameBufferPool(const FrameBufferPool&) = delete;
  FrameBufferPool& operator=(const FrameBufferPool&) = delete;

  // 池空时等待。取消后返回空指针，调用方应停止这一路。
  VideoFrameBufferLease acquire(
      const joint_sensor_player::CancellationToken& cancellation);
  void cancel() noexcept;

  std::size_t allocated_buffers() const noexcept;
  std::size_t in_use() const noexcept;
  std::size_t max_in_use() const noexcept;

 private:
  struct State;
  std::shared_ptr<State> state_;
};

}  // namespace joint_sensor_orchestrator
