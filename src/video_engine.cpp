// 三路视频的线程实现。生产线程和写卡线程参加 StartGate，filler 不参加。
// 写卡线程先睡到 deadline，用醒来时刻判断迟到，然后才 write。
// 有滑动窗口时，pread 在 filler 里，不在写卡线程里。

#include "joint_sensor_orchestrator/video_engine.hpp"

#include "joint_sensor_orchestrator/frame_buffer_pool.hpp"
#include "joint_sensor_orchestrator/thread_affinity.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <limits>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

namespace joint_sensor_orchestrator {

struct VideoEngine::LaneRuntime {
  LaneRuntime(std::size_t capacity, std::size_t frame_bytes)
      : queue(capacity), pool(capacity, frame_bytes) {}

  VideoFrameQueue queue;
  FrameBufferPool pool;
  int fd{-1};
  std::atomic<std::uint64_t> source_frames_scheduled{};
  std::atomic<std::uint64_t> output_frames_written{};
  std::thread producer;
  std::thread consumer;
};

VideoEngine::VideoEngine(OrchestratorConfig config, VideoCache cache,
                         SourceWindow common_window,
                         LifecycleManager& lifecycle,
                         VideoEngineDependencies dependencies)
    : config_(std::move(config)),
      cache_(std::move(cache)),
      common_window_(common_window),
      lifecycle_(lifecycle),
      dependencies_(std::move(dependencies)) {
  if (common_window_.start_ns > common_window_.end_ns) {
    throw std::invalid_argument("视频共同窗口起点晚于终点");
  }
  if (dependencies_.clock == nullptr || dependencies_.sink == nullptr) {
    throw std::invalid_argument("VideoEngine需要Clock和VideoSink");
  }
  if (!dependencies_.pread) {
    dependencies_.pread = [](int fd, void* data, std::size_t size,
                             std::uint64_t offset) {
      if (offset >
          static_cast<std::uint64_t>(
              std::numeric_limits<off_t>::max())) {
        errno = EOVERFLOW;
        return static_cast<std::ptrdiff_t>(-1);
      }
      return static_cast<std::ptrdiff_t>(
          ::pread(fd, data, size, static_cast<off_t>(offset)));
    };
  }
  if (!dependencies_.thread_factory) {
    dependencies_.thread_factory = [](std::function<void()> worker) {
      return std::thread(std::move(worker));
    };
  }
  if (config_.video_prefetch_ms > 0) {
    ahead_ = std::make_unique<VideoAheadCache>(
        cache_,
        static_cast<std::uint64_t>(config_.video_prefetch_ms) * 1'000'000ULL,
        dependencies_.pread);
  }
  for (std::size_t index = 0; index < lanes_.size(); ++index) {
    if (cache_.lanes[index].frame_bytes >
        std::numeric_limits<std::size_t>::max()) {
      throw std::overflow_error("视频帧尺寸超过本机地址空间");
    }
    lanes_[index] = std::make_unique<LaneRuntime>(
        config_.video_queue_frames,
        static_cast<std::size_t>(cache_.lanes[index].frame_bytes));
  }
}

VideoEngine::~VideoEngine() { stop(); }

void VideoEngine::prepare() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != VideoEngineState::Created) {
      throw std::logic_error("VideoEngine只能准备一次");
    }
  }
  try {
    for (std::size_t index = 0; index < lanes_.size(); ++index) {
      const auto& configured = config_.lanes[index];
      const auto& cached = cache_.lanes[index];
      if (configured.name != cached.name ||
          configured.topic != cached.topic ||
          configured.source_fps != cached.source_fps ||
          configured.output_fps != cached.output_fps ||
          cached.frame_count != cached.timestamps_ns.size() ||
          cached.frame_bytes == 0) {
        throw std::runtime_error("视频缓存与lane配置不匹配: " +
                                 configured.name);
      }
      if (cached.frame_count >
          std::numeric_limits<std::uint64_t>::max() /
              cached.frame_bytes) {
        throw std::overflow_error("视频缓存长度溢出");
      }
      const auto expected_size =
          cached.frame_count * cached.frame_bytes;
      const int fd = ::open(cached.payload_path.c_str(), O_RDONLY | O_CLOEXEC);
      if (fd < 0) {
        throw std::runtime_error("无法打开视频缓存: " +
                                 cached.payload_path.string());
      }
      lanes_[index]->fd = fd;
      struct stat info {};
      if (::fstat(fd, &info) != 0 || info.st_size < 0 ||
          static_cast<std::uint64_t>(info.st_size) != expected_size) {
        throw std::runtime_error("视频缓存文件长度错误: " +
                                 cached.payload_path.string());
      }
    }
    const auto prepared = dependencies_.sink->prepare();
    if (!prepared.ok) {
      throw std::runtime_error(prepared.error.empty()
                                   ? "VideoSink准备失败"
                                   : prepared.error);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = VideoEngineState::Ready;
  } catch (const std::exception& error) {
    fail(error.what());
    stop_sink();
    close_files();
    throw;
  }
}

void VideoEngine::arm(std::shared_ptr<StartGate> start_gate) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != VideoEngineState::Ready) {
    throw std::logic_error("VideoEngine必须READY后才能ARMED");
  }
  if (!start_gate) {
    throw std::invalid_argument("VideoEngine需要共享StartGate");
  }
  start_gate_ = std::move(start_gate);
  state_ = VideoEngineState::Armed;
}

void VideoEngine::run() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != VideoEngineState::Armed) {
      throw std::logic_error("VideoEngine必须ARMED后才能RUNNING");
    }
    state_ = VideoEngineState::Running;
  }
  try {
    for (std::size_t index = 0; index < lanes_.size(); ++index) {
      lanes_[index]->producer = dependencies_.thread_factory(
          [this, index] { producer_main(index); });
      lanes_[index]->consumer = dependencies_.thread_factory(
          [this, index] { consumer_main(index); });
    }
    if (ahead_) {
      filler_ = dependencies_.thread_factory([this] { filler_main(); });
    }
  } catch (const std::exception& error) {
    fail(error.what());
    start_gate_->cancel();
    join_workers();
    stop_sink();
    close_files();
    throw;
  }
}

// 参加 StartGate。优先从滑动窗口取已经在内存里的帧，推进本路队列。
// 没有窗口时才在这里 pread，三路共用 xdma_pread_mutex_。
void VideoEngine::producer_main(std::size_t lane_index) noexcept {
  if (lane_index < dependencies_.producer_cpu.size()) {
    pin_current_thread(dependencies_.producer_cpu[lane_index]);
  }
  auto& runtime = *lanes_[lane_index];
  const auto& cached = cache_.lanes[lane_index];
  try {
    StartAnchors anchors;
    if (!start_gate_->arrive_and_wait(
            *lifecycle_.stop_token(), anchors)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!anchors_set_) {
        result_.common_window = common_window_;
        result_.play_start_ns = anchors.play_start_ns;
        result_.utc_anchor_us = anchors.utc_anchor_us;
        anchors_set_ = true;
      }
    }
    const auto slots = build_video_slots(
        cached.timestamps_ns, cached.source_fps, cached.output_fps,
        common_window_, anchors.play_start_ns);
    if (slots.empty()) {
      throw std::runtime_error("共同窗口内没有视频槽: " + cached.name);
    }
    std::size_t slot_index = 0;
    while (slot_index < slots.size()) {
      if (lifecycle_.stop_token()->stop_requested()) {
        cancel_queues();
        return;
      }
      const auto source_index = slots[slot_index].source_index;
      if (source_index >= cached.frame_count ||
          cached.frame_bytes >
              std::numeric_limits<std::size_t>::max()) {
        throw std::runtime_error("视频源帧索引或尺寸越界: " + cached.name);
      }
      if (source_index >
          std::numeric_limits<std::uint64_t>::max() /
              cached.frame_bytes) {
        throw std::overflow_error("视频pread偏移溢出: " + cached.name);
      }
      VideoFrameBufferLease bytes;
      if (ahead_) {
        bytes = ahead_->acquire(lane_index, source_index, [&] {
          return lifecycle_.stop_token()->stop_requested();
        });
        if (!bytes) {
          cancel_queues();
          return;
        }
      } else {
        bytes = runtime.pool.acquire(*lifecycle_.stop_token());
        if (!bytes) return;
      }
      if (!ahead_ && config_.video_backend == VideoBackend::Xdma) {
        if (cached.output_fps == 0) {
          throw std::runtime_error("视频输出帧率无效: " + cached.name);
        }
        // Pace preads one queue-depth ahead of each deadline. During
        // start_delay warmup, also allow filling from the beginning of the
        // delay so serialized ~16MiB reads are not crammed into the last
        // ~queue_depth window (that pattern caused early tx2 lateness).
        // After play_start, keep the tighter pace so radar UDP is not hit
        // by continuous immediate preads.
        const auto queue_lead_ns =
            static_cast<std::uint64_t>(config_.video_queue_frames) *
            (1'000'000'000ULL /
             static_cast<std::uint64_t>(cached.output_fps));
        const auto paced_at_ns =
            slots[slot_index].deadline_ns > queue_lead_ns
                ? slots[slot_index].deadline_ns - queue_lead_ns
                : 0;
        const auto start_delay_ns =
            static_cast<std::uint64_t>(config_.lidar.start_delay_ms) *
            1'000'000ULL;
        const auto warmup_start_ns =
            anchors.play_start_ns > start_delay_ns
                ? anchors.play_start_ns - start_delay_ns
                : 0;
        const auto now_ns = dependencies_.clock->now_ns();
        const auto ready_at_ns =
            now_ns < anchors.play_start_ns
                ? std::min(paced_at_ns, warmup_start_ns)
                : paced_at_ns;
        const auto wait =
            joint_sensor_player::cancellation_aware_sleep_until(
                *dependencies_.clock, ready_at_ns,
                lifecycle_.stop_token().get());
        if (wait == joint_sensor_player::WaitStatus::Cancelled) {
          cancel_queues();
          return;
        }
        if (wait == joint_sensor_player::WaitStatus::Error) {
          throw std::runtime_error("等待视频pread窗口失败: " +
                                   cached.name);
        }
        if (lifecycle_.stop_token()->stop_requested()) {
          cancel_queues();
          return;
        }
        // Serialize lane preads, but copy in ≤1MiB chunks with a yield so a
        // single ~16MiB UYVY read cannot monopolize the CPU for tens of ms
        // and starve the streaming radar sender (seen as ~50-100ms spikes).
        constexpr std::size_t kPreadChunkBytes = 1ULL * 1024 * 1024;
        std::size_t got = 0;
        const auto frame_offset = source_index * cached.frame_bytes;
        while (got < bytes->size()) {
          const auto chunk =
              std::min(kPreadChunkBytes, bytes->size() - got);
          std::ptrdiff_t count = -1;
          {
            std::lock_guard<std::mutex> pread_lock(xdma_pread_mutex_);
            do {
              count = dependencies_.pread(
                  runtime.fd, bytes->data() + got, chunk,
                  frame_offset + got);
            } while (count < 0 && errno == EINTR);
          }
          if (count < 0 ||
              static_cast<std::size_t>(count) != chunk) {
            throw std::runtime_error("视频pread未读取完整帧: " +
                                     cached.name);
          }
          got += static_cast<std::size_t>(count);
          if (got < bytes->size()) {
            std::this_thread::yield();
          }
        }
      }
      ++runtime.source_frames_scheduled;

      std::size_t group_end = slot_index + 1;
      while (group_end < slots.size() &&
             slots[group_end].source_index == source_index) {
        ++group_end;
      }
      std::shared_ptr<const VideoFrameBuffer> shared_bytes = bytes;
      for (; slot_index < group_end; ++slot_index) {
        if (!runtime.queue.push(
                {slots[slot_index], shared_bytes})) {
          return;
        }
      }
      if (ahead_) ahead_->release_before(lane_index, source_index + 1);
    }
    runtime.queue.close();
  } catch (const std::exception& error) {
    fail(error.what());
  } catch (...) {
    fail("视频生产者发生未知异常");
  }
}

// 参加 StartGate。睡到这一槽的绝对期限后先采样迟到，再把整帧交给 VideoSink。
// 这里的 actual_ns 是醒来时刻。write 花了多久不记进这一帧的迟到。
void VideoEngine::consumer_main(std::size_t lane_index) noexcept {
  if (lane_index < dependencies_.consumer_cpu.size()) {
    pin_current_thread(dependencies_.consumer_cpu[lane_index]);
  }
  auto& runtime = *lanes_[lane_index];
  const auto& cached = cache_.lanes[lane_index];
  try {
    StartAnchors anchors;
    if (!start_gate_->arrive_and_wait(
            *lifecycle_.stop_token(), anchors)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!anchors_set_) {
        result_.common_window = common_window_;
        result_.play_start_ns = anchors.play_start_ns;
        result_.utc_anchor_us = anchors.utc_anchor_us;
        anchors_set_ = true;
      }
    }
    const auto slots = build_video_slots(
        cached.timestamps_ns, cached.source_fps, cached.output_fps,
        common_window_, anchors.play_start_ns);
    if (slots.empty()) {
      throw std::runtime_error("共同窗口内没有视频槽: " + cached.name);
    }
    for (std::size_t expected = 0; expected < slots.size();
         ++expected) {
      if (lifecycle_.stop_token()->stop_requested()) {
        cancel_queues();
        return;
      }
      auto frame = runtime.queue.pop();
      if (!frame.has_value()) {
        if (lifecycle_.stop_token()->stop_requested()) return;
        throw std::runtime_error("视频队列断供: " + cached.name);
      }
      if (frame->slot.output_index != expected) {
        throw std::runtime_error("视频输出槽顺序错误: " + cached.name);
      }
      const auto wait = joint_sensor_player::cancellation_aware_sleep_until(
          *dependencies_.clock, frame->slot.deadline_ns,
          lifecycle_.stop_token().get());
      if (wait == joint_sensor_player::WaitStatus::Cancelled) {
        cancel_queues();
        return;
      }
      if (wait == joint_sensor_player::WaitStatus::Error) {
        throw std::runtime_error("等待视频绝对期限失败: " + cached.name);
      }
      if (lifecycle_.stop_token()->stop_requested()) {
        cancel_queues();
        return;
      }
      const auto actual_ns = dependencies_.clock->now_ns();
      if (actual_ns > frame->slot.deadline_ns &&
          actual_ns - frame->slot.deadline_ns >
              static_cast<std::uint64_t>(
                  config_.video_max_lateness_us) *
                  1000ULL) {
        throw std::runtime_error("视频迟到超过门限: " + cached.name);
      }
      if (!frame->bytes) {
        throw std::runtime_error("视频队列包含空帧缓冲: " + cached.name);
      }
      const auto written = dependencies_.sink->write_frame(
          cached.name, *frame->bytes, frame->slot, actual_ns);
      if (!written.ok) {
        throw std::runtime_error(
            written.error.empty() ? "VideoSink写入失败" : written.error);
      }
      if (dependencies_.frame_observer) {
        dependencies_.frame_observer(
            cached.name, frame->slot, actual_ns,
            actual_ns > frame->slot.deadline_ns
                ? actual_ns - frame->slot.deadline_ns
                : 0);
      }
      ++runtime.output_frames_written;
    }
  } catch (const std::exception& error) {
    fail(error.what());
  } catch (...) {
    fail("视频消费者发生未知异常");
  }
}

// 不参加 StartGate。开播前先把窗口装满，之后哪一路先缺帧就先补哪一路。
// 读到最后一帧真正有输出槽的源帧就返回，播完不会卡在 join。
void VideoEngine::filler_main() noexcept {
  pin_current_thread(dependencies_.filler_cpu);
  try {
    // A source frame can sit inside the common window and still have no
    // output slot (a 10 Hz repeat group whose last grid is past the window).
    // Those frames are never released by the producer. Stop at the last
    // scheduled source index so the filler returns when playback is done.
    std::array<std::uint64_t, 3> scheduled_end{};
    for (std::size_t lane = 0; lane < cache_.lanes.size(); ++lane) {
      const auto& cached = cache_.lanes[lane];
      const auto slots = build_video_slots(
          cached.timestamps_ns, cached.source_fps, cached.output_fps,
          common_window_, 0);
      scheduled_end[lane] =
          slots.empty() ? 0 : slots.back().source_index + 1;
    }
    while (!lifecycle_.stop_token()->stop_requested() &&
           ahead_ && !ahead_->stopped()) {
      bool any_lane = false;
      std::optional<std::size_t> shortest;
      std::uint64_t shortest_missing = std::numeric_limits<std::uint64_t>::max();
      for (std::size_t lane = 0; lane < cache_.lanes.size(); ++lane) {
        auto next = ahead_->next_index(lane);
        const auto& stamps = cache_.lanes[lane].timestamps_ns;
        while (next < stamps.size() && next < scheduled_end[lane] &&
               stamps[next] < common_window_.start_ns) {
          ahead_->release_before(lane, next + 1);
          next = ahead_->next_index(lane);
        }
        if (next >= stamps.size() || next >= scheduled_end[lane] ||
            stamps[next] > common_window_.end_ns) {
          continue;
        }
        any_lane = true;
        // Serve whichever lane's next unread frame is soonest, so the
        // 30 Hz lane is not filled only after the 10 Hz windows are full.
        const auto missing =
            ahead_->next_missing_ns(lane, scheduled_end[lane]);
        if (!missing || *missing > common_window_.end_ns) continue;
        if (*missing < shortest_missing) {
          shortest = lane;
          shortest_missing = *missing;
        }
      }
      if (!any_lane) return;
      bool progressed = false;
      if (shortest) {
        const auto next = ahead_->next_index(*shortest);
        const auto& stamps = cache_.lanes[*shortest].timestamps_ns;
        progressed = ahead_->prefetch_one(
            *shortest, stamps[next],
            [&] { return lifecycle_.stop_token()->stop_requested(); },
            scheduled_end[*shortest]);
      }
      if (!progressed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  } catch (const std::exception& error) {
    if (ahead_) ahead_->request_stop();
    fail(error.what());
  } catch (...) {
    if (ahead_) ahead_->request_stop();
    fail("视频预读发生未知异常");
  }
}

void VideoEngine::fail(std::string reason) noexcept {
  if (ahead_) ahead_->request_stop();
  bool first = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (result_.error.empty()) {
      result_.error = reason.empty() ? "VideoEngine失败" : std::move(reason);
      state_ = VideoEngineState::Failed;
      first = true;
    }
  }
  if (first) {
    try {
      lifecycle_.fail("video", result_.error);
    } catch (...) {
    }
  }
  if (start_gate_) start_gate_->cancel();
  cancel_queues();
}

void VideoEngine::cancel_queues() noexcept {
  for (auto& lane : lanes_) {
    lane->queue.cancel();
    lane->pool.cancel();
  }
}

void VideoEngine::join_workers() noexcept {
  if (filler_.joinable()) filler_.join();
  for (auto& lane : lanes_) {
    if (lane->producer.joinable()) lane->producer.join();
    if (lane->consumer.joinable()) lane->consumer.join();
  }
}

void VideoEngine::close_files() noexcept {
  for (auto& lane : lanes_) {
    if (lane->fd >= 0) {
      ::close(lane->fd);
      lane->fd = -1;
    }
  }
}

void VideoEngine::stop_sink() noexcept {
  bool should_stop = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sink_stopped_) {
      sink_stopped_ = true;
      should_stop = true;
    }
  }
  if (should_stop) dependencies_.sink->stop();
}

void VideoEngine::stop() noexcept {
  if (ahead_) ahead_->request_stop();
  lifecycle_.request_stop();
  if (start_gate_) start_gate_->cancel();
  cancel_queues();
  join_workers();
  stop_sink();
  close_files();
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != VideoEngineState::Failed) {
    state_ = VideoEngineState::Stopped;
  }
}

VideoEngineResult VideoEngine::result() {
  join_workers();
  stop_sink();
  close_files();
  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t index = 0; index < lanes_.size(); ++index) {
    result_.lanes[index] = {
        cache_.lanes[index].name,
        lanes_[index]->source_frames_scheduled.load(),
        lanes_[index]->output_frames_written.load(),
        lanes_[index]->queue.max_waterline(),
        lanes_[index]->pool.allocated_buffers(),
        lanes_[index]->pool.max_in_use()};
  }
  result_.cancelled =
      lifecycle_.stop_token()->stop_requested() && result_.error.empty();
  result_.ok = result_.error.empty();
  if (state_ != VideoEngineState::Failed) {
    state_ = VideoEngineState::Stopped;
  }
  return result_;
}

VideoEngineState VideoEngine::state() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

}  // namespace joint_sensor_orchestrator
