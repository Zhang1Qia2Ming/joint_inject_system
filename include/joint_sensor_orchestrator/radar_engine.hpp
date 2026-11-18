#pragma once

// 八路雷达的发送侧。一个工作线程参加 StartGate，拿到锚点后按包的绝对期限发 UDP。
// 读 mcap 的线程不参加这道门。迟到在 send 之前判断，和视频写卡前采样是同一规矩。
// PTC 只在开播前应答标定请求，不占发送时间。

#include "joint_sensor_orchestrator/lifecycle_manager.hpp"
#include "joint_sensor_orchestrator/ptp_schedule.hpp"
#include "joint_sensor_orchestrator/start_gate.hpp"
#include "joint_sensor_orchestrator/unified_timeline.hpp"
#include "joint_sensor_player/mcap.hpp"
#include "joint_sensor_player/player.hpp"
#include "joint_sensor_player/udp_sender.hpp"

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace joint_sensor_orchestrator {

enum class RadarEngineState {
  Created,
  Ready,
  Armed,
  Running,
  Stopped,
  Failed,
};

class RadarPtcService {
 public:
  virtual ~RadarPtcService() = default;
  virtual void start() = 0;
  virtual void stop() noexcept = 0;
};

using RadarScanFunction = std::function<joint_sensor_player::McapCatalog(
    const std::filesystem::path&,
    const std::vector<joint_sensor_player::LidarChannelConfig>&)>;
using RadarLightweightScanFunction =
    std::function<joint_sensor_player::LightweightMcapCatalog(
        const std::filesystem::path&,
        const std::vector<joint_sensor_player::LidarChannelConfig>&)>;

struct StreamingRadarLimits {
  std::size_t packet_queue_capacity{8192};
  std::size_t frame_reorder_capacity{128};
};

struct StreamingRadarPreflight {
  std::size_t required_frame_reorder{};
  std::size_t required_packet_buffer{};
};

using RadarFrameStreamFunction = std::function<
    joint_sensor_player::McapStreamStats(
        const joint_sensor_player::McapFrameObserver&)>;

std::uint64_t earliest_possible_frame_deadline_ns(
    const joint_sensor_player::PlayerConfig& config,
    std::uint64_t common_source_start_ns,
    std::uint64_t frame_bag_log_time_ns,
    std::uint64_t play_start_ns);

bool packet_deadline_is_safe(
    std::uint64_t packet_deadline_ns,
    std::uint64_t unread_frame_deadline_lower_bound_ns) noexcept;

StreamingRadarPreflight analyze_streaming_radar(
    const joint_sensor_player::PlayerConfig& config,
    const joint_sensor_player::LightweightMcapCatalog& catalog,
    SourceWindow common_window);

// 边读 mcap 边发。点云不整包进内存。迟到在每个包 send 之前采样。
joint_sensor_player::PlaybackResult play_streaming_radar(
    const joint_sensor_player::PlayerConfig& config,
    const joint_sensor_player::LightweightMcapCatalog& catalog,
    SourceWindow common_window, joint_sensor_player::Clock& clock,
    joint_sensor_player::DatagramSink& sink,
    joint_sensor_player::CancellationToken& cancellation,
    std::uint64_t utc_anchor_us, std::uint64_t play_start_ns,
    const RadarFrameStreamFunction& stream,
    joint_sensor_player::FrameStatObserver frame_observer = {},
    StreamingRadarLimits limits = {},
    int reader_cpu = -1,
    std::optional<PtpPlaybackPlan> ptp_plan = std::nullopt);

struct RadarEngineDependencies {
  joint_sensor_player::Clock* clock{};
  joint_sensor_player::DatagramSink* sink{};
  std::unique_ptr<RadarPtcService> ptc;
  std::optional<joint_sensor_player::McapCatalog> prevalidated_catalog;
  std::optional<joint_sensor_player::LightweightMcapCatalog>
      prevalidated_lightweight_catalog;
  RadarScanFunction scan;
  RadarLightweightScanFunction lightweight_scan;
  RadarFrameStreamFunction stream;
  StreamingRadarLimits streaming_limits;
  ThreadFactory thread_factory;
  joint_sensor_player::FrameStatObserver frame_observer;
  // -1 表示不绑核。发送线程绑到网卡队列所在的核上会把首秒雷达迟到拉大。
  int sender_cpu{-1};
  int reader_cpu{-1};
  // 空表示沿用现有期限。打开后由 worker 在拿到 play_start 时自己算位移。
  std::optional<PtpPlaybackPlan> ptp_plan;
};

struct RadarEngineResult {
  joint_sensor_player::PlaybackResult playback;
  SourceWindow common_window;
  std::uint64_t utc_anchor_us{};
};

class RadarEngine {
 public:
  RadarEngine(joint_sensor_player::PlayerConfig config,
              SourceWindow common_window, LifecycleManager& lifecycle,
              RadarEngineDependencies dependencies);
  ~RadarEngine();
  RadarEngine(const RadarEngine&) = delete;
  RadarEngine& operator=(const RadarEngine&) = delete;

  void prepare();
  void arm(std::shared_ptr<StartGate> start_gate);
  void run();
  void stop() noexcept;
  RadarEngineResult result();
  RadarEngineState state() const noexcept;

 private:
  // 参加 StartGate。醒来后按包期限发送，迟到超过门限就停。
  void worker_main() noexcept;
  void stop_ptc() noexcept;

  joint_sensor_player::PlayerConfig config_;
  SourceWindow common_window_;
  LifecycleManager& lifecycle_;
  RadarEngineDependencies dependencies_;
  joint_sensor_player::McapCatalog catalog_;
  joint_sensor_player::LightweightMcapCatalog lightweight_catalog_;
  bool streaming_{};
  std::unique_ptr<joint_sensor_player::LidarPacketRewriter> rewriter_;
  std::shared_ptr<StartGate> start_gate_;
  RadarEngineResult result_;
  mutable std::mutex mutex_;
  RadarEngineState state_{RadarEngineState::Created};
  std::thread worker_;
};

}  // namespace joint_sensor_orchestrator
