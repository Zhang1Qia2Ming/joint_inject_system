#include "joint_sensor_orchestrator/frame_buffer_pool.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>

namespace joint_sensor_orchestrator {

struct FrameBufferPool::State {
  State(std::size_t capacity, std::size_t frame_bytes)
      : frame_bytes(frame_bytes), buffers(capacity), free_indices(capacity) {
    std::iota(free_indices.begin(), free_indices.end(), 0);
  }

  void release(std::size_t index) noexcept {
    {
      std::lock_guard<std::mutex> lock(mutex);
      free_indices.push_back(index);
      --in_use;
    }
    available.notify_one();
  }

  mutable std::mutex mutex;
  std::condition_variable available;
  const std::size_t frame_bytes;
  std::vector<std::unique_ptr<VideoFrameBuffer>> buffers;
  std::vector<std::size_t> free_indices;
  std::size_t allocated{};
  std::size_t in_use{};
  std::size_t max_in_use{};
  bool cancelled{};
};

FrameBufferPool::FrameBufferPool(std::size_t capacity,
                                 std::size_t frame_bytes)
    : state_(std::make_shared<State>(capacity, frame_bytes)) {
  if (capacity == 0 || frame_bytes == 0) {
    throw std::invalid_argument("视频帧池容量和帧尺寸必须大于零");
  }
}

FrameBufferPool::~FrameBufferPool() { cancel(); }

VideoFrameBufferLease FrameBufferPool::acquire(
    const joint_sensor_player::CancellationToken& cancellation) {
  std::unique_lock<std::mutex> lock(state_->mutex);
  while (state_->free_indices.empty() && !state_->cancelled &&
         !cancellation.stop_requested()) {
    state_->available.wait_for(lock, std::chrono::milliseconds(1));
  }
  if (state_->cancelled || cancellation.stop_requested()) return {};
  const auto index = state_->free_indices.back();
  state_->free_indices.pop_back();
  if (!state_->buffers[index]) {
    state_->buffers[index] =
        std::make_unique<VideoFrameBuffer>(state_->frame_bytes);
    ++state_->allocated;
  }
  ++state_->in_use;
  state_->max_in_use = std::max(state_->max_in_use, state_->in_use);
  std::weak_ptr<State> weak = state_;
  return VideoFrameBufferLease(
      state_->buffers[index].get(), [weak, index](VideoFrameBuffer*) noexcept {
        if (auto state = weak.lock()) state->release(index);
      });
}

void FrameBufferPool::cancel() noexcept {
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->cancelled = true;
  }
  state_->available.notify_all();
}

std::size_t FrameBufferPool::allocated_buffers() const noexcept {
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->allocated;
}

std::size_t FrameBufferPool::in_use() const noexcept {
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->in_use;
}

std::size_t FrameBufferPool::max_in_use() const noexcept {
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->max_in_use;
}

}  // namespace joint_sensor_orchestrator
