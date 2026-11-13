#include "joint_sensor_orchestrator/enter_wait.hpp"
#include "joint_sensor_orchestrator/playback_status.hpp"
#include "joint_sensor_orchestrator/smoke_radar.hpp"
#include "test_framework.hpp"

#include <atomic>
#include <chrono>
#include <thread>

#include <unistd.h>

namespace {

class FakeVideo final : public joint_sensor_orchestrator::OrchestratedComponent {
 public:
  explicit FakeVideo(joint_sensor_orchestrator::LifecycleManager& lifecycle)
      : lifecycle_(lifecycle) {}

  void prepare() override {}
  void arm(std::shared_ptr<joint_sensor_orchestrator::StartGate> gate) override {
    gate_ = std::move(gate);
  }
  void run() override {
    for (int i = 0; i < 6; ++i) {
      threads_.emplace_back([this] {
        joint_sensor_orchestrator::StartAnchors anchors;
        if (!gate_->arrive_and_wait(*lifecycle_.stop_token(), anchors)) return;
        released = true;
        observed = anchors;
      });
    }
  }
  void wait() override {
    for (auto& thread : threads_) {
      if (thread.joinable()) thread.join();
    }
  }
  void stop() noexcept override { wait(); }
  bool ok() const override { return true; }
  std::string error() const override { return {}; }

  joint_sensor_orchestrator::LifecycleManager& lifecycle_;
  std::shared_ptr<joint_sensor_orchestrator::StartGate> gate_;
  std::vector<std::thread> threads_;
  std::atomic<bool> released{};
  joint_sensor_orchestrator::StartAnchors observed;
};

}  // namespace

TEST_CASE(smoke_radar_participates_in_seven_worker_start_gate) {
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::SmokeRadarComponent radar(lifecycle);
  FakeVideo video(lifecycle);
  joint_sensor_orchestrator::OrchestratorOptions options;
  options.start_delay_ns = 100;
  options.gate_timeout = std::chrono::milliseconds(500);
  options.monotonic_now_ns = [] { return 1'000ULL; };
  options.realtime_now_us = [] { return 2'000ULL; };
  joint_sensor_orchestrator::Orchestrator orchestrator(
      lifecycle, radar, video, std::move(options));
  const auto result = orchestrator.run();
  CHECK(result.ok);
  CHECK(radar.ok());
  CHECK(video.released.load());
  CHECK(video.observed.play_start_ns > 0);
}

TEST_CASE(finalize_run_status_duration_limit_is_pass_not_incomplete) {
  const auto status = joint_sensor_orchestrator::finalize_run_status(
      true, true, {}, joint_sensor_orchestrator::StopIntent::DurationLimit,
      true);
  CHECK(status.ok);
  CHECK(!status.cancelled);
  CHECK(status.stop_reason == "duration_limit");
  CHECK(status.software_logic ==
        joint_sensor_orchestrator::ValidationStatus::Pass);
  CHECK(status.video_payload_io ==
        joint_sensor_orchestrator::ValidationStatus::Pass);
  CHECK(status.xdma_device ==
        joint_sensor_orchestrator::ValidationStatus::Pass);
}

TEST_CASE(finalize_run_status_signal_is_incomplete) {
  const auto status = joint_sensor_orchestrator::finalize_run_status(
      true, true, {}, joint_sensor_orchestrator::StopIntent::Signal, true);
  CHECK(status.ok);
  CHECK(status.cancelled);
  CHECK(status.stop_reason == "signal");
  CHECK(status.software_logic ==
        joint_sensor_orchestrator::ValidationStatus::Incomplete);
  CHECK(status.xdma_device ==
        joint_sensor_orchestrator::ValidationStatus::Incomplete);
}

TEST_CASE(finalize_run_status_keeps_mock_payload_layers_not_run) {
  const auto status = joint_sensor_orchestrator::finalize_run_status(
      true, false, {}, joint_sensor_orchestrator::StopIntent::None, false);
  CHECK(status.ok);
  CHECK(status.software_logic ==
        joint_sensor_orchestrator::ValidationStatus::Pass);
  CHECK(status.video_payload_io ==
        joint_sensor_orchestrator::ValidationStatus::NotRun);
  CHECK(status.xdma_device ==
        joint_sensor_orchestrator::ValidationStatus::NotRun);
}

TEST_CASE(playback_status_distinguishes_idle_from_sending) {
  using joint_sensor_orchestrator::PlaybackPhase;
  using joint_sensor_orchestrator::format_phase_status;
  const auto preparing = format_phase_status(PlaybackPhase::PreparingVideo);
  CHECK(preparing.find("正在准备视频窗口") != std::string::npos);
  CHECK(preparing.find("尚未发送") != std::string::npos);
  const auto waiting = format_phase_status(
      PlaybackPhase::WaitingToSend, std::chrono::milliseconds(25000));
  CHECK(waiting.find("等待开播") != std::string::npos);
  CHECK(waiting.find("25.000s") != std::string::npos);
  CHECK(waiting.find("尚未发送") != std::string::npos);
  const auto enter_wait =
      format_phase_status(PlaybackPhase::WaitingForEnter);
  CHECK(enter_wait.find("等待回车") != std::string::npos);
  CHECK(enter_wait.find("按下后才预热队列") != std::string::npos);
  CHECK(enter_wait.find("尚未发送") != std::string::npos);
  const auto prefill = format_phase_status(
      PlaybackPhase::Prefilling, std::chrono::milliseconds(25000));
  CHECK(prefill.find("正在预填发送队列") != std::string::npos);
  CHECK(prefill.find("25.000s") != std::string::npos);
  CHECK(prefill.find("尚未发送") != std::string::npos);
  const auto sending = format_phase_status(
      PlaybackPhase::Sending, std::chrono::milliseconds(3200),
      std::chrono::milliseconds(10000));
  CHECK(sending.find("发送中") != std::string::npos);
  CHECK(sending.find("3.200s") != std::string::npos);
  CHECK(sending.find("10.000s") != std::string::npos);
  CHECK(sending.find("尚未发送") == std::string::npos);
}

TEST_CASE(enter_wait_ignores_newline_typed_before_prompt) {
  int fds[2] = {-1, -1};
  CHECK(::pipe(fds) == 0);
  CHECK(::write(fds[1], "\n", 1) == 1);
  joint_sensor_orchestrator::discard_pending_input(fds[0]);
  ::close(fds[1]);
  const auto result = joint_sensor_orchestrator::wait_for_newline(
      fds[0], [] { return false; });
  CHECK(result == joint_sensor_orchestrator::EnterWaitResult::Closed);
  ::close(fds[0]);
}

TEST_CASE(enter_wait_accepts_newline_typed_after_prompt) {
  int fds[2] = {-1, -1};
  CHECK(::pipe(fds) == 0);
  CHECK(::write(fds[1], "\n", 1) == 1);
  joint_sensor_orchestrator::discard_pending_input(fds[0]);
  ssize_t written = -1;
  std::thread later([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const char line[] = "\n";
    written = ::write(fds[1], line, 1);
  });
  const auto result = joint_sensor_orchestrator::wait_for_newline(
      fds[0], [] { return false; });
  later.join();
  CHECK(written == 1);
  CHECK(result == joint_sensor_orchestrator::EnterWaitResult::Confirmed);
  ::close(fds[0]);
  ::close(fds[1]);
}

TEST_CASE(playback_status_marks_start_end_and_duration) {
  const auto start = std::chrono::system_clock::time_point{};
  const auto end = start + std::chrono::milliseconds(10007);
  const auto started = joint_sensor_orchestrator::format_send_start(start);
  const auto finished =
      joint_sensor_orchestrator::format_send_end(start, end);
  CHECK(started.find("发送开始: ") == 0);
  CHECK(finished.find("发送结束: ") == 0);
  CHECK(finished.find("发送时长: 10.007s") != std::string::npos);
}

TEST_CASE(finalize_run_status_engine_failure_beats_duration_intent) {
  const auto status = joint_sensor_orchestrator::finalize_run_status(
      false, true, "视频迟到超过门限: tx2",
      joint_sensor_orchestrator::StopIntent::DurationLimit, true);
  CHECK(!status.ok);
  CHECK(status.stop_reason == "视频迟到超过门限: tx2");
  CHECK(status.software_logic ==
        joint_sensor_orchestrator::ValidationStatus::Fail);
  CHECK(status.xdma_device ==
        joint_sensor_orchestrator::ValidationStatus::Fail);
}
