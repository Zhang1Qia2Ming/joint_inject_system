#pragma once

#include "joint_sensor_orchestrator/lifecycle_manager.hpp"
#include "joint_sensor_orchestrator/start_gate.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace joint_sensor_orchestrator {

inline constexpr std::size_t kOrchestratorWorkerCount = 7;

class OrchestratedComponent {
 public:
  virtual ~OrchestratedComponent() = default;
  virtual void prepare() = 0;
  virtual void arm(std::shared_ptr<StartGate> gate) = 0;
  virtual void run() = 0;
  virtual void wait() = 0;
  virtual void stop() noexcept = 0;
  virtual bool ok() const = 0;
  virtual std::string error() const = 0;
};

struct OrchestratorOptions {
  std::uint64_t start_delay_ns{2'000'000'000ULL};
  std::chrono::milliseconds gate_timeout{30'000};
  std::function<std::uint64_t()> monotonic_now_ns;
  std::function<std::uint64_t()> realtime_now_us;
  std::function<void()> before_release;
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

  OrchestratorResult run() noexcept;

 private:
  LifecycleManager& lifecycle_;
  OrchestratedComponent& radar_;
  OrchestratedComponent& video_;
  OrchestratorOptions options_;
};

}  // namespace joint_sensor_orchestrator
