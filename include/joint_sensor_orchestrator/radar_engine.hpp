#pragma once

#include "joint_sensor_orchestrator/lifecycle_manager.hpp"
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
    int reader_cpu = -1);

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
  int sender_cpu{-1};
  int reader_cpu{-1};
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
