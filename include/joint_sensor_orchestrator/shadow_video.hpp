#pragma once

// Sensor4 到 Sensor7 的主机侧替身。读原生 1920x1200 UYVY，按 30 Hz 网格等待，
// 然后丢掉这一帧。没有 VideoSink，也不打开 /dev/xdma。
// 排期帧在开播前全部读进内存，发送期间不再读盘。

#include "joint_sensor_orchestrator/start_gate.hpp"
#include "joint_sensor_orchestrator/unified_timeline.hpp"
#include "joint_sensor_player/player.hpp"

#include <array>
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
  // 这条路径不写卡，正常运行保持 0。
  std::uint64_t h2c_writes{};
};

struct ShadowVideoResult {
  bool ok{false};
  std::string error;
  std::vector<ShadowLaneResult> lanes;
};

class ShadowVideo {
 public:
  ShadowVideo(std::vector<ShadowLaneInput> lanes, SourceWindow window,
              joint_sensor_player::Clock& clock,
  std::shared_ptr<joint_sensor_player::CancellationToken> stop,
  std::uint64_t prefetch_ns,
  std::array<int, 4> cpus = {-1, -1, -1, -1});
  ~ShadowVideo();

  ShadowVideo(const ShadowVideo&) = delete;
  ShadowVideo& operator=(const ShadowVideo&) = delete;

  // 每路一条线程。先绑核并读完排期帧，再等 notify_play_start。
  void start();
  // 等到每一路的排期帧都在内存里再返回。发送期间这四路不再读盘。
  bool wait_until_preloaded();
  // 由编排器在 StartGate 放出锚点后调用。四路从这里开始按网格睡觉。
  void notify_play_start(const StartAnchors& anchors);
  ShadowVideoResult wait();

 private:
  struct State;
  std::unique_ptr<State> state_;
};

std::vector<ShadowLaneInput> load_shadow_lanes(
    const std::filesystem::path& manifest_path);

}  // namespace joint_sensor_orchestrator
