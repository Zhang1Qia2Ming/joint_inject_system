#include "joint_sensor_orchestrator/video_frame_queue.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace joint_sensor_orchestrator {

VideoFrameQueue::VideoFrameQueue(std::size_t capacity) : capacity_(capacity) {
  if (capacity < 3 || capacity > 6) {
    throw std::invalid_argument("视频队列容量必须在 3 到 6 之间");
  }
}

bool VideoFrameQueue::push(QueuedVideoFrame frame) {
  std::unique_lock<std::mutex> lock(mutex_);
  not_full_.wait(lock, [this] {
    return frames_.size() < capacity_ || closed_ || cancelled_;
  });
  if (closed_ || cancelled_) {
    return false;
  }
  frames_.push_back(std::move(frame));
  max_waterline_ = std::max(max_waterline_, frames_.size());
  lock.unlock();
  not_empty_.notify_one();
  return true;
}

std::optional<QueuedVideoFrame> VideoFrameQueue::pop() {
  std::unique_lock<std::mutex> lock(mutex_);
  not_empty_.wait(lock, [this] {
    return !frames_.empty() || closed_ || cancelled_;
  });
  if (cancelled_ || frames_.empty()) {
    return std::nullopt;
  }
  QueuedVideoFrame frame = std::move(frames_.front());
  frames_.pop_front();
  lock.unlock();
  not_full_.notify_one();
  return frame;
}

void VideoFrameQueue::close() noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    closed_ = true;
  }
  not_empty_.notify_all();
  not_full_.notify_all();
}

void VideoFrameQueue::cancel() noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelled_ = true;
  }
  not_empty_.notify_all();
  not_full_.notify_all();
}

std::size_t VideoFrameQueue::size() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return frames_.size();
}

std::size_t VideoFrameQueue::max_waterline() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return max_waterline_;
}

bool VideoFrameQueue::eof() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return closed_ && frames_.empty();
}

}  // namespace joint_sensor_orchestrator
