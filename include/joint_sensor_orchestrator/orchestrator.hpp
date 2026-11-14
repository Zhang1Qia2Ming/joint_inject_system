#pragma once

// 一次联合播放的总控。雷达和视频各自准备好后，在同一道 StartGate 上对齐。
// 门打开时交出 play_start 和 UTC 锚点，两边都按这个绝对时间发送。
// 参加这道门的是 3 个视频生产线程、3 个视频写卡线程和 1 个雷达线程。
// 预读 filler 和四路环视模拟不参加，磁盘读不会挡住开播。

#include "joint_sensor_orchestrator/lifecycle_manager.hpp"
#include "joint_sensor_orchestrator/start_gate.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace joint_sensor_orchestrator {

// 3 个视频生产线程 + 3 个视频写卡线程 + 1 个雷达线程。
inline constexpr std::size_t kOrchestratorWorkerCount = 7;

// 雷达和视频都实现这组动作。prepare 只做打开和检查，run 才真正起线程。
class OrchestratedComponent {
 public:
  virtual ~OrchestratedComponent() = default;
  virtual void prepare() = 0;
  // 拿到共享的 StartGate，线程起来后要在这道门上等。
  virtual void arm(std::shared_ptr<StartGate> gate) = 0;
  virtual void run() = 0;
  // 等到本组件的工作线程结束。
  virtual void wait() = 0;
  virtual void stop() noexcept = 0;
  virtual bool ok() const = 0;
  virtual std::string error() const = 0;
};

struct OrchestratorOptions {
  // 门打开后再等这么久才是第一帧的期限。这段时间用来把发送队列预填满。
  std::uint64_t start_delay_ns{2'000'000'000ULL};
  std::chrono::milliseconds gate_timeout{30'000};
  std::function<std::uint64_t()> monotonic_now_ns;
  std::function<std::uint64_t()> realtime_now_us;
  // 全部门齐、尚未放出锚点时调用。用来做开播前的最后一次准备。
  std::function<void()> before_release;
  // 锚点已经交给各线程之后立刻调用。环视模拟在这里收到 play_start。
  std::function<void(const StartAnchors&)> on_play_start;
};

struct OrchestratorResult {
  struct StateTiming {
    LifecycleState state{LifecycleState::Created};
    std::uint64_t entered_monotonic_ns{};
    std::uint64_t duration_ns{};
  };

  bool ok{};
  bool cancelled{};
  StartAnchors anchors;
  LifecycleSnapshot lifecycle;
  std::vector<StateTiming> timeline;
  std::string error;
};

class Orchestrator {
 public:
  Orchestrator(LifecycleManager& lifecycle, OrchestratedComponent& radar,
               OrchestratedComponent& video,
               OrchestratorOptions options = {});

  // 准备、对齐、放出锚点、等两边跑完。失败也把生命周期记进结果里。
  OrchestratorResult run() noexcept;

 private:
  LifecycleManager& lifecycle_;
  OrchestratedComponent& radar_;
  OrchestratedComponent& video_;
  OrchestratorOptions options_;
};

}  // namespace joint_sensor_orchestrator
