// 视频冒烟时的雷达占位。它占掉 StartGate 的雷达名额，到达后马上离开。

#include "joint_sensor_orchestrator/smoke_radar.hpp"

#include <stdexcept>
#include <utility>

namespace joint_sensor_orchestrator {

SmokeRadarComponent::SmokeRadarComponent(LifecycleManager& lifecycle)
    : lifecycle_(lifecycle) {}

SmokeRadarComponent::~SmokeRadarComponent() { stop(); }

void SmokeRadarComponent::prepare() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (prepared_ || stopped_) {
    throw std::logic_error("SmokeRadar只能准备一次");
  }
  prepared_ = true;
}

void SmokeRadarComponent::arm(std::shared_ptr<StartGate> gate) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!prepared_ || armed_ || !gate) {
    throw std::logic_error("SmokeRadar必须READY后持有StartGate再ARMED");
  }
  start_gate_ = std::move(gate);
  armed_ = true;
}

void SmokeRadarComponent::run() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!armed_ || worker_.joinable()) {
      throw std::logic_error("SmokeRadar必须ARMED后才能RUNNING");
    }
  }
  try {
    worker_ = std::thread([this] { worker_main(); });
  } catch (const std::exception& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    failed_ = true;
    error_ = error.what();
    throw;
  }
}

// 占住雷达那个名额。到达 StartGate 后不发送，直接返回。
void SmokeRadarComponent::worker_main() noexcept {
  try {
    StartAnchors anchors;
    (void)start_gate_->arrive_and_wait(*lifecycle_.stop_token(), anchors);
  } catch (const std::exception& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    failed_ = true;
    error_ = error.what();
    try {
      lifecycle_.fail("radar", error_);
    } catch (...) {
    }
    if (start_gate_) start_gate_->cancel();
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    failed_ = true;
    error_ = "SmokeRadar工作线程未知异常";
    try {
      lifecycle_.fail("radar", error_);
    } catch (...) {
    }
    if (start_gate_) start_gate_->cancel();
  }
}

void SmokeRadarComponent::wait() {
  if (worker_.joinable()) worker_.join();
}

void SmokeRadarComponent::stop() noexcept {
  lifecycle_.request_stop();
  if (start_gate_) start_gate_->cancel();
  if (worker_.joinable()) worker_.join();
  std::lock_guard<std::mutex> lock(mutex_);
  stopped_ = true;
}

bool SmokeRadarComponent::ok() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return !failed_;
}

std::string SmokeRadarComponent::error() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return error_;
}

FinalRunStatus finalize_run_status(bool engines_ok, bool engines_cancelled,
                                   const std::string& engines_error,
                                   StopIntent intent, bool xdma_backend) {
  FinalRunStatus status;
  const auto fill_xdma = [&](ValidationStatus value) {
    if (!xdma_backend) return;
    status.video_payload_io = value;
    status.xdma_device = value;
  };

  if (!engines_ok) {
    status.ok = false;
    status.cancelled = false;
    status.stop_reason =
        engines_error.empty() ? "联合播放失败" : engines_error;
    status.software_logic = ValidationStatus::Fail;
    fill_xdma(ValidationStatus::Fail);
    return status;
  }

  if (intent == StopIntent::Signal) {
    status.ok = true;
    status.cancelled = true;
    status.stop_reason = "signal";
    status.software_logic = ValidationStatus::Incomplete;
    fill_xdma(ValidationStatus::Incomplete);
    return status;
  }

  if (intent == StopIntent::DurationLimit) {
    status.ok = true;
    status.cancelled = false;
    status.stop_reason = "duration_limit";
    status.software_logic = ValidationStatus::Pass;
    fill_xdma(ValidationStatus::Pass);
    return status;
  }

  if (engines_cancelled) {
    status.ok = true;
    status.cancelled = true;
    status.stop_reason = "signal";
    status.software_logic = ValidationStatus::Incomplete;
    fill_xdma(ValidationStatus::Incomplete);
    return status;
  }

  status.ok = true;
  status.cancelled = false;
  status.stop_reason = "completed";
  status.software_logic = ValidationStatus::Pass;
  fill_xdma(ValidationStatus::Pass);
  return status;
}

}  // namespace joint_sensor_orchestrator
