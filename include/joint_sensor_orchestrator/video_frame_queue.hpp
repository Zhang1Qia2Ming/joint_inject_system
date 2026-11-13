#pragma once

#include "joint_sensor_orchestrator/frame_buffer_pool.hpp"
#include "joint_sensor_orchestrator/unified_timeline.hpp"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace joint_sensor_orchestrator {

struct QueuedVideoFrame {
  VideoSlot slot;
  std::shared_ptr<const VideoFrameBuffer> bytes;
};

class VideoFrameQueue {
 public:
  explicit VideoFrameQueue(std::size_t capacity);

  bool push(QueuedVideoFrame frame);
  std::optional<QueuedVideoFrame> pop();
  void close() noexcept;
  void cancel() noexcept;

  std::size_t size() const noexcept;
  std::size_t max_waterline() const noexcept;
  bool eof() const noexcept;

 private:
  const std::size_t capacity_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_;
  std::condition_variable not_full_;
  std::deque<QueuedVideoFrame> frames_;
  std::size_t max_waterline_{};
  bool cancelled_{};
  bool closed_{};
};

}  // namespace joint_sensor_orchestrator
