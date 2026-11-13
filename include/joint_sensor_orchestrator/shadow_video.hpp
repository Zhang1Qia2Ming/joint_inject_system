#pragma once

#include "joint_sensor_orchestrator/start_gate.hpp"
#include "joint_sensor_orchestrator/unified_timeline.hpp"
#include "joint_sensor_player/player.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace joint_sensor_orchestrator {

struct ShadowLaneInput {
  std::string name;
  std::filesystem::path payload_path;
  std::vector<std::uint64_t> timestamps_ns;
  std::uint64_t frame_bytes{};
  unsigned source_fps{10};
  unsigned output_fps{30};
};

struct ShadowLaneResult {
  std::string name;
  std::uint64_t outputs{};
  std::uint64_t frames_read{};
  std::uint64_t max_lateness_ns{};
  std::uint64_t h2c_writes{};
};

struct ShadowVideoResult {
  bool ok{false};
  std::string error;
  std::vector<ShadowLaneResult> lanes;
};

// Host-side stand-in for Sensor4-7. It reads native UYVY and waits on the
// same 30 Hz grid as the real lanes, then drops the frame. It has no H2C
// device and must not open /dev/xdma*.
class ShadowVideo {
 public:
  ShadowVideo(std::vector<ShadowLaneInput> lanes, SourceWindow window,
              joint_sensor_player::Clock& clock,
              std::shared_ptr<joint_sensor_player::CancellationToken> stop,
              std::uint64_t prefetch_ns);
  ~ShadowVideo();

  ShadowVideo(const ShadowVideo&) = delete;
  ShadowVideo& operator=(const ShadowVideo&) = delete;

  void start();
  void notify_play_start(const StartAnchors& anchors);
  ShadowVideoResult wait();

 private:
  struct State;
  std::unique_ptr<State> state_;
};

std::vector<ShadowLaneInput> load_shadow_lanes(
    const std::filesystem::path& manifest_path);

}  // namespace joint_sensor_orchestrator
