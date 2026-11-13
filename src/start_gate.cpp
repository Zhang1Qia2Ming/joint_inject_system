#include "joint_sensor_orchestrator/start_gate.hpp"

#include <stdexcept>

namespace joint_sensor_orchestrator {

StartGate::StartGate(std::size_t expected_workers)
    : expected_workers_(expected_workers) {
  if (expected_workers == 0) {
    throw std::invalid_argument("StartGate预期线程数必须大于零");
  }
}

bool StartGate::arrive_and_wait(
    const joint_sensor_player::CancellationToken& cancellation,
    StartAnchors& anchors) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (released_ || cancelled_ || ready_workers_ >= expected_workers_) {
    return false;
  }
  ++ready_workers_;
  changed_.notify_all();
  while (!released_ && !cancelled_ && !cancellation.stop_requested()) {
    changed_.wait_for(lock, std::chrono::milliseconds(1));
  }
  if (!released_ || cancelled_ || cancellation.stop_requested()) {
    return false;
  }
  anchors = anchors_;
  return true;
}

bool StartGate::wait_until_ready(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  return changed_.wait_for(lock, timeout, [this] {
    return ready_workers_ == expected_workers_ || cancelled_;
  }) &&
         ready_workers_ == expected_workers_ && !cancelled_;
}

void StartGate::release(StartAnchors anchors) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (cancelled_ || released_ ||
        ready_workers_ != expected_workers_) {
      throw std::logic_error(
          "StartGate只能在全部线程READY后释放一次");
    }
    anchors_ = anchors;
    released_ = true;
  }
  changed_.notify_all();
}

void StartGate::cancel() noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelled_ = true;
  }
  changed_.notify_all();
}

std::size_t StartGate::ready_count() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return ready_workers_;
}

bool StartGate::released() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return released_;
}

}  // namespace joint_sensor_orchestrator
