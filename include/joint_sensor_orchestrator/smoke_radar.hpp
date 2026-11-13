#pragma once

#include "joint_sensor_orchestrator/lifecycle_manager.hpp"
#include "joint_sensor_orchestrator/orchestrator.hpp"
#include "joint_sensor_orchestrator/report_v2.hpp"
#include "joint_sensor_orchestrator/start_gate.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace joint_sensor_orchestrator {

// Video-only smoke participant: arrives at the shared StartGate as the
// single radar worker, then exits without sending UDP/PTC.
class SmokeRadarComponent final : public OrchestratedComponent {
 public:
  explicit SmokeRadarComponent(LifecycleManager& lifecycle);
  ~SmokeRadarComponent() override;

  void prepare() override;
  void arm(std::shared_ptr<StartGate> gate) override;
  void run() override;
  void wait() override;
  void stop() noexcept override;
  bool ok() const override;
  std::string error() const override;

 private:
  void worker_main() noexcept;

  LifecycleManager& lifecycle_;
  std::shared_ptr<StartGate> start_gate_;
  std::thread worker_;
  mutable std::mutex mutex_;
  bool prepared_{};
  bool armed_{};
  bool stopped_{};
  bool failed_{};
  std::string error_;
};

enum class StopIntent {
  None,
  Signal,
  DurationLimit,
};

struct FinalRunStatus {
  bool ok{};
  bool cancelled{};
  std::string stop_reason;
  ValidationStatus software_logic{ValidationStatus::NotRun};
  ValidationStatus video_payload_io{ValidationStatus::NotRun};
  ValidationStatus xdma_device{ValidationStatus::NotRun};
};

FinalRunStatus finalize_run_status(bool engines_ok, bool engines_cancelled,
                                   const std::string& engines_error,
                                   StopIntent intent, bool xdma_backend);

}  // namespace joint_sensor_orchestrator
