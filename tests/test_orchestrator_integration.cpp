#include "joint_sensor_orchestrator/orchestrator.hpp"
#include "test_framework.hpp"

#include <atomic>
#include <chrono>
#include <thread>

namespace {

class FakeComponent final
    : public joint_sensor_orchestrator::OrchestratedComponent {
 public:
  FakeComponent(std::string name, std::size_t workers,
                joint_sensor_orchestrator::LifecycleManager& lifecycle)
      : name_(std::move(name)), workers_(workers), lifecycle_(lifecycle) {}

  void prepare() override {
    ++prepare_calls;
    if (prepare_failure) throw std::runtime_error(name_ + " prepare failed");
    ready = true;
  }

  void arm(std::shared_ptr<joint_sensor_orchestrator::StartGate> gate) override {
    CHECK(ready);
    gate_ = std::move(gate);
    armed = true;
  }

  void run() override {
    CHECK(armed);
    for (std::size_t index = 0; index < workers_; ++index) {
      threads_.emplace_back([this] {
        joint_sensor_orchestrator::StartAnchors anchors;
        if (!gate_->arrive_and_wait(*lifecycle_.stop_token(), anchors)) return;
        observed = anchors;
        running = true;
        if (runtime_failure && !failed.exchange(true)) {
          lifecycle_.fail(name_, name_ + " runtime failed");
        }
      });
    }
  }

  void wait() override {
    for (auto& thread : threads_) {
      if (thread.joinable()) thread.join();
    }
  }

  void stop() noexcept override {
    ++stop_calls;
    wait();
    stopped = true;
  }

  bool ok() const override { return !prepare_failure && !runtime_failure; }
  std::string error() const override {
    if (prepare_failure) return name_ + " prepare failed";
    if (runtime_failure) return name_ + " runtime failed";
    return {};
  }

  std::string name_;
  std::size_t workers_;
  joint_sensor_orchestrator::LifecycleManager& lifecycle_;
  std::shared_ptr<joint_sensor_orchestrator::StartGate> gate_;
  std::vector<std::thread> threads_;
  std::atomic<unsigned> prepare_calls{};
  std::atomic<unsigned> stop_calls{};
  std::atomic<bool> running{};
  std::atomic<bool> failed{};
  bool prepare_failure{};
  bool runtime_failure{};
  bool ready{};
  bool armed{};
  bool stopped{};
  joint_sensor_orchestrator::StartAnchors observed;
};

joint_sensor_orchestrator::OrchestratorOptions immediate_options() {
  joint_sensor_orchestrator::OrchestratorOptions options;
  options.start_delay_ns = 200;
  options.gate_timeout = std::chrono::milliseconds(200);
  options.monotonic_now_ns = [] { return 1'000ULL; };
  options.realtime_now_us = [] { return 2'000ULL; };
  return options;
}

}  // namespace

TEST_CASE(orchestrator_releases_exactly_seven_workers_with_shared_anchors) {
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  FakeComponent radar("radar", 1, lifecycle);
  FakeComponent video("video", 6, lifecycle);
  auto options = immediate_options();
  std::atomic<std::uint64_t> now{1'000};
  options.monotonic_now_ns = [&] { return now.fetch_add(100); };
  joint_sensor_orchestrator::Orchestrator orchestrator(
      lifecycle, radar, video, std::move(options));

  const auto result = orchestrator.run();

  CHECK(result.ok);
  CHECK(!result.cancelled);
  CHECK(radar.observed.play_start_ns > 1'200);
  CHECK(video.observed.play_start_ns == radar.observed.play_start_ns);
  CHECK(video.observed.utc_anchor_us == 2'000);
  CHECK(lifecycle.state() ==
        joint_sensor_orchestrator::LifecycleState::Stopped);
  CHECK(radar.stop_calls.load() == 1);
  CHECK(video.stop_calls.load() == 1);
  CHECK(!result.timeline.empty());
  CHECK(result.timeline.front().state ==
        joint_sensor_orchestrator::LifecycleState::Created);
  CHECK(result.timeline.front().duration_ns > 0);
}

TEST_CASE(orchestrator_prepare_failure_never_runs_other_components) {
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  FakeComponent radar("radar", 1, lifecycle);
  FakeComponent video("video", 6, lifecycle);
  radar.prepare_failure = true;
  joint_sensor_orchestrator::Orchestrator orchestrator(
      lifecycle, radar, video, immediate_options());

  const auto result = orchestrator.run();

  CHECK(!result.ok);
  CHECK(!video.running.load());
  CHECK(lifecycle.failure().component == "radar");
  CHECK(lifecycle.state() ==
        joint_sensor_orchestrator::LifecycleState::Stopped);
  CHECK(radar.stop_calls.load() == 1);
  CHECK(video.stop_calls.load() == 1);
}

TEST_CASE(orchestrator_video_prepare_failure_keeps_video_as_first_root) {
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  FakeComponent radar("radar", 1, lifecycle);
  FakeComponent video("video", 6, lifecycle);
  video.prepare_failure = true;
  joint_sensor_orchestrator::Orchestrator orchestrator(
      lifecycle, radar, video, immediate_options());

  const auto result = orchestrator.run();

  CHECK(!result.ok);
  CHECK(lifecycle.failure().component == "video");
  CHECK(!radar.running.load());
  CHECK(!video.running.load());
}

TEST_CASE(orchestrator_runtime_failure_globally_stops_and_keeps_first_root) {
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  FakeComponent radar("radar", 1, lifecycle);
  FakeComponent video("video", 6, lifecycle);
  video.runtime_failure = true;
  joint_sensor_orchestrator::Orchestrator orchestrator(
      lifecycle, radar, video, immediate_options());

  const auto result = orchestrator.run();

  CHECK(!result.ok);
  CHECK(lifecycle.failure().component == "video");
  CHECK(lifecycle.stop_token()->stop_requested());
  CHECK(radar.stopped);
  CHECK(video.stopped);
}

TEST_CASE(orchestrator_ctrl_c_style_cancellation_is_normal_stopped) {
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  FakeComponent radar("radar", 1, lifecycle);
  FakeComponent video("video", 6, lifecycle);
  auto options = immediate_options();
  options.before_release = [&] { lifecycle.request_stop(); };
  joint_sensor_orchestrator::Orchestrator orchestrator(
      lifecycle, radar, video, std::move(options));

  const auto result = orchestrator.run();

  CHECK(result.ok);
  CHECK(result.cancelled);
  CHECK(lifecycle.failure().component.empty());
  CHECK(lifecycle.state() ==
        joint_sensor_orchestrator::LifecycleState::Stopped);
}
