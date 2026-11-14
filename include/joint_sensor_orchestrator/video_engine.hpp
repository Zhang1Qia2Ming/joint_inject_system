#pragma once

// 三路真实视频的调度。
// filler 把 SSD 上的帧读进滑动内存窗口，不参加 StartGate。
// 每路一个 producer，从窗口取帧推进本路队列，参加 StartGate。
// 每路一个 consumer，睡到绝对期限，采样迟到，再把整帧交给 VideoSink。

#include "joint_sensor_orchestrator/lifecycle_manager.hpp"
#include "joint_sensor_orchestrator/start_gate.hpp"
#include "joint_sensor_orchestrator/orchestrator_config.hpp"
#include "joint_sensor_orchestrator/unified_timeline.hpp"
#include "joint_sensor_orchestrator/video_ahead_cache.hpp"
#include "joint_sensor_orchestrator/video_cache.hpp"
#include "joint_sensor_orchestrator/video_frame_queue.hpp"
#include "joint_sensor_orchestrator/video_sink.hpp"

#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace joint_sensor_orchestrator {

enum class VideoEngineState {
  Created,
  Ready,
  Armed,
  Running,
  Stopped,
  Failed,
};

using VideoPreadFunction =
    std::function<std::ptrdiff_t(int, void*, std::size_t, std::uint64_t)>;
// actual_write_ns 是写卡前的醒来时刻，不是 write 返回的时刻。
using VideoFrameObserver = std::function<void(
    const std::string&, const VideoSlot&, std::uint64_t actual_write_ns,
    std::uint64_t lateness_ns)>;

struct VideoEngineDependencies {
  joint_sensor_player::Clock* clock{};
  VideoSink* sink{};
  VideoPreadFunction pread;
  ThreadFactory thread_factory;
  VideoFrameObserver frame_observer;
  // -1 leaves the thread unpinned. Set only for the hardware send path.
  std::array<int, 3> consumer_cpu{-1, -1, -1};
  std::array<int, 3> producer_cpu{-1, -1, -1};
  int filler_cpu{-1};
};

struct VideoLaneResult {
  std::string name;
  std::uint64_t source_frames_scheduled{};
  std::uint64_t output_frames_written{};
  std::size_t max_queue_waterline{};
  std::size_t allocated_buffers{};
  std::size_t max_buffers_in_use{};
};

struct VideoEngineResult {
  bool ok{};
  bool cancelled{};
  std::string error;
  SourceWindow common_window;
  std::uint64_t play_start_ns{};
  std::uint64_t utc_anchor_us{};
  std::array<VideoLaneResult, 3> lanes;
};

class VideoEngine {
 public:
  VideoEngine(OrchestratorConfig config, VideoCache cache,
              SourceWindow common_window, LifecycleManager& lifecycle,
              VideoEngineDependencies dependencies);
  ~VideoEngine();
  VideoEngine(const VideoEngine&) = delete;
  VideoEngine& operator=(const VideoEngine&) = delete;

  void prepare();
  void arm(std::shared_ptr<StartGate> start_gate);
  void run();
  void stop() noexcept;
  VideoEngineResult result();
  VideoEngineState state() const noexcept;

 private:
  struct LaneRuntime;

  // 参加 StartGate。从预读窗口取帧推进本路队列。
  // 没有预读窗口时才在这里 pread，三路共用一把锁。
  void producer_main(std::size_t lane_index) noexcept;
  // 不参加 StartGate。窗口装满后继续向前读，读到最后一帧有输出槽的源帧就停。
  void filler_main() noexcept;
  // 参加 StartGate。睡到期限，先采样迟到，再把整帧交给 VideoSink。
  void consumer_main(std::size_t lane_index) noexcept;
  void fail(std::string reason) noexcept;
  void cancel_queues() noexcept;
  void join_workers() noexcept;
  void close_files() noexcept;
  void stop_sink() noexcept;

  OrchestratorConfig config_;
  VideoCache cache_;
  SourceWindow common_window_;
  LifecycleManager& lifecycle_;
  VideoEngineDependencies dependencies_;
  std::shared_ptr<StartGate> start_gate_;
  std::unique_ptr<VideoAheadCache> ahead_;
  std::thread filler_;
  std::array<std::unique_ptr<LaneRuntime>, 3> lanes_;
  mutable std::mutex mutex_;
  std::mutex xdma_pread_mutex_;
  VideoEngineState state_{VideoEngineState::Created};
  VideoEngineResult result_;
  bool sink_stopped_{};
  bool anchors_set_{};
};

}  // namespace joint_sensor_orchestrator
