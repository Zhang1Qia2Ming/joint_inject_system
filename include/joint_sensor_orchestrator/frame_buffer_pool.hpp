#pragma once

#include "joint_sensor_player/player.hpp"

#include <stdlib.h>

#include <cstddef>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <vector>

namespace joint_sensor_orchestrator {

// O_DIRECT needs a 4096-byte aligned destination. The default allocator
// only guarantees a much smaller alignment.
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
