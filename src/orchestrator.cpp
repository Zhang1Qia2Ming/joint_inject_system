#include "joint_sensor_orchestrator/orchestrator.hpp"

#include <chrono>
#include <limits>
#include <stdexcept>
#include <utility>

namespace joint_sensor_orchestrator {
namespace {

std::uint64_t steady_now_ns() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

std::uint64_t system_now_us() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

std::uint64_t checked_add(std::uint64_t left, std::uint64_t right,
                          const char* message) {
  if (left > std::numeric_limits<std::uint64_t>::max() - right) {
    throw std::overflow_error(message);
  }
  return left + right;
}

}  // namespace

Orchestrator::Orchestrator(LifecycleManager& lifecycle,
                           OrchestratedComponent& radar,
                           OrchestratedComponent& video,
                           OrchestratorOptions options)
    : lifecycle_(lifecycle),
      radar_(radar),
      video_(video),
      options_(std::move(options)) {
  if (!options_.monotonic_now_ns) options_.monotonic_now_ns = steady_now_ns;
  if (!options_.realtime_now_us) options_.realtime_now_us = system_now_us;
  if (options_.gate_timeout.count() <= 0) {
    throw std::invalid_argument("StartGate等待超时必须大于零");
  }
}

OrchestratorResult Orchestrator::run() noexcept {
  OrchestratorResult result;
  std::string active_component{"orchestrator"};
  result.timeline.push_back(
      {LifecycleState::Created, options_.monotonic_now_ns(), 0});
  const auto note_state = [&](LifecycleState state) {
    if (result.timeline.back().state == state) return;
    const auto now = options_.monotonic_now_ns();
    result.timeline.back().duration_ns =
        now - result.timeline.back().entered_monotonic_ns;
    result.timeline.push_back({state, now, 0});
  };
  const auto transition = [&](LifecycleState state) {
    lifecycle_.transition(state);
    note_state(state);
  };
  auto gate = std::make_shared<StartGate>(kOrchestratorWorkerCount);
  bool stopped = false;
  const auto stop_all = [&] {
    if (stopped) return;
    stopped = true;
    lifecycle_.request_stop();
    gate->cancel();
    radar_.stop();
    video_.stop();
  };
  try {
    transition(LifecycleState::Preparing);
    active_component = "radar";
    radar_.prepare();
    active_component = "video";
    video_.prepare();
    active_component = "orchestrator";
    transition(LifecycleState::Ready);

    radar_.arm(gate);
    video_.arm(gate);
    transition(LifecycleState::Armed);
    radar_.run();
    video_.run();
    if (!gate->wait_until_ready(options_.gate_timeout)) {
      throw std::runtime_error("7个联合工作线程未全部到达StartGate");
    }
    if (options_.before_release) options_.before_release();
    result.anchors.play_start_ns =
        checked_add(options_.monotonic_now_ns(), options_.start_delay_ns,
                    "联合单调时钟起点溢出");
    result.anchors.utc_anchor_us =
        checked_add(options_.realtime_now_us(),
                    options_.start_delay_ns / 1000ULL,
                    "联合UTC锚点溢出");
    gate->release(result.anchors);
    if (options_.on_play_start) options_.on_play_start(result.anchors);
    transition(LifecycleState::Running);

    radar_.wait();
    video_.wait();
    if (!radar_.ok() || !video_.ok()) {
      const auto component = !radar_.ok() ? "radar" : "video";
      const auto reason = !radar_.ok() ? radar_.error() : video_.error();
      if (lifecycle_.failure().component.empty()) {
        lifecycle_.fail(component,
                        reason.empty() ? "组件运行失败" : reason);
      }
    }
  } catch (const std::exception& error) {
    result.error = error.what();
    if (lifecycle_.failure().component.empty()) {
      try {
        lifecycle_.fail(active_component, result.error);
      } catch (...) {
      }
    }
  } catch (...) {
    result.error = "协调器发生未知异常";
    if (lifecycle_.failure().component.empty()) {
      try {
        lifecycle_.fail("orchestrator", result.error);
      } catch (...) {
      }
    }
  }

  const bool cancelled_before_stop =
      lifecycle_.stop_token()->stop_requested() &&
      lifecycle_.failure().component.empty();
  const auto state_before_stop = lifecycle_.state();
  note_state(state_before_stop);
  try {
    if (state_before_stop == LifecycleState::Running ||
        state_before_stop == LifecycleState::Failed) {
      transition(LifecycleState::Stopping);
    }
  } catch (...) {
  }
  stop_all();
  try {
    if (lifecycle_.state() == LifecycleState::Stopping) {
      transition(LifecycleState::Stopped);
    }
  } catch (...) {
  }

  result.lifecycle = lifecycle_.snapshot();
  const auto finished_at = options_.monotonic_now_ns();
  result.timeline.back().duration_ns =
      finished_at - result.timeline.back().entered_monotonic_ns;
  result.cancelled = cancelled_before_stop;
  result.ok = result.lifecycle.first_failure.component.empty();
  if (!result.ok && result.error.empty()) {
    result.error = result.lifecycle.first_failure.reason;
  }
  return result;
}

}  // namespace joint_sensor_orchestrator
