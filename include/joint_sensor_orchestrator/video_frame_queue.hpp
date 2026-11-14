#pragma once

// 一路视频从生产线程到写卡线程的队列。容量就是配置里的 video_queue_frames。
// 每路自己一把锁。队列里有帧时，取出只占很短的临界区，锁在唤醒对方之前放开。
// 空队列会让写卡线程停住，下一拍的间隔会跳一大截。

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

  // 队列满时等到有空位。取消或关闭返回 false。
  bool push(QueuedVideoFrame frame);
  // 队列空时等到有帧。关闭且空了返回 nullopt。
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
