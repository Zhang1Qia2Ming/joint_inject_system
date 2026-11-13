#include "joint_sensor_orchestrator/video_engine.hpp"
#include "test_framework.hpp"

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

class VideoFakeClock final : public joint_sensor_player::Clock {
 public:
  std::uint64_t now_ns() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return per_thread_now_[std::this_thread::get_id()];
  }
  bool sleep_until_ns(std::uint64_t deadline_ns) override {
    std::lock_guard<std::mutex> lock(mutex_);
    per_thread_now_[std::this_thread::get_id()] =
        deadline_ns + late_by_ns;
    return true;
  }

  std::uint64_t late_by_ns{};

 private:
  mutable std::mutex mutex_;
  mutable std::unordered_map<std::thread::id, std::uint64_t>
      per_thread_now_;
};

class VideoCancellingClock final : public joint_sensor_player::Clock {
 public:
  std::uint64_t now_ns() const override { return now_.load(); }
  bool sleep_until_ns(std::uint64_t deadline_ns) override {
    const auto before = now_.exchange(deadline_ns);
    const auto step = deadline_ns > before ? deadline_ns - before : 0;
    auto observed = max_step_ns.load();
    while (step > observed &&
           !max_step_ns.compare_exchange_weak(observed, step)) {
    }
    cancellation->request_stop();
    return true;
  }

  joint_sensor_player::CancellationToken* cancellation{};
  std::atomic<std::uint64_t> max_step_ns{};

 private:
  std::atomic<std::uint64_t> now_{};
};

class RecordingVideoSink final
    : public joint_sensor_orchestrator::VideoSink {
 public:
  joint_sensor_orchestrator::VideoSinkResult prepare() override {
    prepared = true;
    if (prepare_fail) return {false, "video prepare failed"};
    return {true, {}};
  }

  joint_sensor_orchestrator::VideoSinkResult write_frame(
      const std::string& lane, std::span<const std::byte> frame,
      const joint_sensor_orchestrator::VideoSlot& slot,
      std::uint64_t) override {
    std::lock_guard<std::mutex> lock(mutex);
    if (lane == fail_lane && slot.output_index == fail_output) {
      health_value.ok = false;
      health_value.error = "video sink failed";
      return {false, health_value.error};
    }
    slots[lane].push_back(slot);
    bytes[lane].push_back(std::to_integer<unsigned char>(frame.front()));
    payload_addresses[lane].push_back(frame.data());
    ++health_value.frames_written;
    health_value.bytes_written += frame.size();
    return {true, {}};
  }

  joint_sensor_orchestrator::VideoSinkHealth health() const override {
    std::lock_guard<std::mutex> lock(mutex);
    return health_value;
  }

  void stop() noexcept override { stopped = true; }

  mutable std::mutex mutex;
  bool prepared{};
  bool prepare_fail{};
  std::atomic<bool> stopped{};
  std::string fail_lane;
  std::uint64_t fail_output{std::numeric_limits<std::uint64_t>::max()};
  std::unordered_map<
      std::string, std::vector<joint_sensor_orchestrator::VideoSlot>>
      slots;
  std::unordered_map<std::string, std::vector<unsigned char>> bytes;
  std::unordered_map<std::string, std::vector<const std::byte*>>
      payload_addresses;
  joint_sensor_orchestrator::VideoSinkHealth health_value;
};

struct VideoFixture {
  VideoFixture() {
    root = std::filesystem::temp_directory_path() /
           ("video-engine-" + std::to_string(::getpid()) + "-" +
            std::to_string(next.fetch_add(1)));
    std::filesystem::create_directories(root);
  }
  ~VideoFixture() { std::filesystem::remove_all(root); }

  joint_sensor_orchestrator::VideoCache cache() const {
    joint_sensor_orchestrator::VideoCache value;
    const std::array<std::string, 3> names{"tx1", "tx2", "tx4"};
    const std::array<unsigned, 3> rates{10, 30, 10};
    for (std::size_t index = 0; index < names.size(); ++index) {
      auto& lane = value.lanes[index];
      lane.name = names[index];
      lane.topic = "/camera/" + names[index];
      lane.source_fps = rates[index];
      lane.output_fps = 30;
      lane.frame_count = 2;
      lane.frame_bytes = 4;
      lane.payload_path = root / (names[index] + ".uyvy");
      lane.timestamps_ns =
          rates[index] == 30
              ? std::vector<std::uint64_t>{0, 33'333'333}
              : std::vector<std::uint64_t>{0, 100'000'000};
      std::ofstream output(lane.payload_path, std::ios::binary);
      const std::array<char, 8> payload{
          static_cast<char>(10 + index), 1, 2, 3,
          static_cast<char>(20 + index), 4, 5, 6};
      output.write(payload.data(), payload.size());
    }
    return value;
  }

  static std::atomic<unsigned> next;
  std::filesystem::path root;
};

std::atomic<unsigned> VideoFixture::next{};

joint_sensor_orchestrator::OrchestratorConfig video_config() {
  joint_sensor_orchestrator::OrchestratorConfig config;
  config.video_queue_frames = 3;
  config.video_max_lateness_us = 5000;
  config.lanes = {{
      {"tx1", "/camera/tx1", 0, 10, 30},
      {"tx2", "/camera/tx2", 1, 30, 30},
      {"tx4", "/camera/tx4", 2, 10, 30},
  }};
  return config;
}

}  // namespace

TEST_CASE(video_engine_serializes_xdma_preads_across_lanes) {
  VideoFixture fixture;
  VideoFakeClock clock;
  RecordingVideoSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  std::atomic<int> in_flight{0};
  std::atomic<int> max_in_flight{0};
  joint_sensor_orchestrator::VideoEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.pread = [&](int fd, void* data, std::size_t size,
                           std::uint64_t offset) {
    const int current = in_flight.fetch_add(1) + 1;
    int observed = max_in_flight.load();
    while (current > observed &&
           !max_in_flight.compare_exchange_weak(observed, current)) {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const auto count =
        ::pread(fd, data, size, static_cast<off_t>(offset));
    in_flight.fetch_sub(1);
    return count;
  };
  auto config = video_config();
  config.video_backend = joint_sensor_orchestrator::VideoBackend::Xdma;
  joint_sensor_orchestrator::VideoEngine engine(
      config, fixture.cache(), {0, 199'999'999}, lifecycle,
      std::move(dependencies));
  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(6);
  engine.arm(gate);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  gate->release({1'000'000'000ULL, 5'000'000});
  const auto result = engine.result();
  CHECK(result.ok);
  CHECK(max_in_flight.load() == 1);
}

TEST_CASE(video_engine_prefills_xdma_during_start_delay_then_paces) {
  VideoFixture fixture;
  VideoFakeClock clock;
  RecordingVideoSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  std::mutex pread_mutex;
  std::vector<std::uint64_t> pread_now_ns;
  joint_sensor_orchestrator::VideoEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.pread = [&](int fd, void* data, std::size_t size,
                           std::uint64_t offset) {
    {
      std::lock_guard<std::mutex> lock(pread_mutex);
      pread_now_ns.push_back(clock.now_ns());
    }
    return ::pread(fd, data, size, static_cast<off_t>(offset));
  };
  auto config = video_config();
  config.video_backend = joint_sensor_orchestrator::VideoBackend::Xdma;
  config.video_queue_frames = 3;
  config.lidar.start_delay_ms = 5000;
  joint_sensor_orchestrator::VideoEngine engine(
      config, fixture.cache(), {0, 199'999'999}, lifecycle,
      std::move(dependencies));
  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(6);
  engine.arm(gate);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  const std::uint64_t play_start_ns = 10'000'000'000ULL;
  gate->release({play_start_ns, 5'000'000});
  const auto result = engine.result();
  CHECK(result.ok);
  const auto queue_lead_ns =
      static_cast<std::uint64_t>(config.video_queue_frames) *
      (1'000'000'000ULL / 30ULL);
  const auto start_delay_ns =
      static_cast<std::uint64_t>(config.lidar.start_delay_ms) *
      1'000'000ULL;
  CHECK(!pread_now_ns.empty());
  const auto earliest =
      *std::min_element(pread_now_ns.begin(), pread_now_ns.end());
  // Warmup: first unique frames may load from the start of start_delay.
  CHECK(earliest <= play_start_ns - start_delay_ns);
  CHECK(earliest < play_start_ns - queue_lead_ns);
}

TEST_CASE(video_engine_preads_sources_once_and_preserves_repeat_identity) {
  VideoFixture fixture;
  VideoFakeClock clock;
  RecordingVideoSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  std::atomic<unsigned> pread_calls{};
  joint_sensor_orchestrator::VideoEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.pread = [&](int fd, void* data, std::size_t size,
                           std::uint64_t offset) {
    ++pread_calls;
    return ::pread(fd, data, size, static_cast<off_t>(offset));
  };
  std::atomic<unsigned> evidence_count{};
  dependencies.frame_observer =
      [&](const std::string&, const joint_sensor_orchestrator::VideoSlot&,
          std::uint64_t, std::uint64_t) { ++evidence_count; };
  auto config = video_config();
  config.video_backend = joint_sensor_orchestrator::VideoBackend::Xdma;
  joint_sensor_orchestrator::VideoEngine engine(
      config, fixture.cache(), {0, 199'999'999}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  CHECK(engine.state() ==
        joint_sensor_orchestrator::VideoEngineState::Ready);
  CHECK(sink.slots.empty());
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(6);
  engine.arm(gate);
  CHECK(engine.state() ==
        joint_sensor_orchestrator::VideoEngineState::Armed);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  CHECK(sink.slots.empty());
  gate->release({1'000, 5'000'000});
  const auto result = engine.result();

  CHECK(result.ok);
  CHECK(result.utc_anchor_us == 5'000'000);
  CHECK(pread_calls.load() == 6);
  CHECK(sink.slots["tx1"].size() == 6);
  CHECK(sink.slots["tx2"].size() == 2);
  CHECK(sink.slots["tx4"].size() == 6);
  CHECK(evidence_count.load() == 14);
  CHECK(sink.slots["tx1"][0].source_index == 0);
  CHECK(sink.slots["tx1"][1].source_index == 0);
  CHECK(sink.slots["tx1"][2].source_index == 0);
  CHECK(sink.bytes["tx1"][0] == sink.bytes["tx1"][1]);
  CHECK(sink.bytes["tx1"][1] == sink.bytes["tx1"][2]);
  CHECK(sink.payload_addresses["tx1"][0] ==
        sink.payload_addresses["tx1"][1]);
  CHECK(sink.payload_addresses["tx1"][1] ==
        sink.payload_addresses["tx1"][2]);
  for (const auto& lane : result.lanes) {
    CHECK(lane.max_queue_waterline <= 3);
    CHECK(lane.allocated_buffers > 0);
    CHECK(lane.allocated_buffers <= 3);
    CHECK(lane.max_buffers_in_use <= 3);
  }
  CHECK(engine.state() ==
        joint_sensor_orchestrator::VideoEngineState::Stopped);
  CHECK(sink.stopped.load());
}

TEST_CASE(mock_video_runs_forty_second_equivalent_without_pread) {
  VideoFixture fixture;
  auto cache = fixture.cache();
  constexpr std::size_t ten_hz_frames = 400;
  constexpr std::size_t thirty_hz_frames = 1200;
  for (std::size_t lane_index = 0; lane_index < cache.lanes.size();
       ++lane_index) {
    auto& lane = cache.lanes[lane_index];
    const auto count = lane.source_fps == 10 ? ten_hz_frames
                                             : thirty_hz_frames;
    lane.frame_count = count;
    lane.timestamps_ns.clear();
    for (std::size_t index = 0; index < count; ++index) {
      lane.timestamps_ns.push_back(
          static_cast<std::uint64_t>(index) *
          1'000'000'000ULL / lane.source_fps);
    }
    std::filesystem::resize_file(
        lane.payload_path, lane.frame_count * lane.frame_bytes);
  }
  VideoFakeClock clock;
  RecordingVideoSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  std::atomic<unsigned> pread_calls{};
  joint_sensor_orchestrator::VideoEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.pread = [&](int, void*, std::size_t,
                           std::uint64_t) -> std::ptrdiff_t {
    ++pread_calls;
    return -1;
  };
  joint_sensor_orchestrator::VideoEngine engine(
      video_config(), std::move(cache), {0, 39'900'000'000ULL}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(6);
  engine.arm(gate);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  gate->release({1'000, 5'000'000});
  const auto result = engine.result();

  CHECK(result.ok);
  CHECK(pread_calls.load() == 0);
  CHECK(result.lanes[0].source_frames_scheduled == 399);
  CHECK(result.lanes[1].source_frames_scheduled == 1198);
  CHECK(result.lanes[2].source_frames_scheduled == 399);
  CHECK(result.lanes[0].output_frames_written == 1197);
  CHECK(result.lanes[1].output_frames_written == 1198);
  CHECK(result.lanes[2].output_frames_written == 1197);
  CHECK(result.lanes[0].output_frames_written ==
        result.lanes[0].source_frames_scheduled * 3);
  CHECK(result.lanes[2].output_frames_written ==
        result.lanes[2].source_frames_scheduled * 3);
  for (const auto& lane : result.lanes) {
    CHECK(lane.max_queue_waterline <= 3);
  }
}

TEST_CASE(real_mock_sink_runs_forty_second_equivalent_end_to_end) {
  VideoFixture fixture;
  auto cache = fixture.cache();
  for (auto& lane : cache.lanes) {
    const std::size_t count = lane.source_fps == 10 ? 400 : 1200;
    lane.frame_count = count;
    lane.frame_bytes = joint_sensor_orchestrator::kVideoFrameBytes;
    lane.timestamps_ns.clear();
    for (std::size_t index = 0; index < count; ++index) {
      lane.timestamps_ns.push_back(
          static_cast<std::uint64_t>(index) *
          1'000'000'000ULL / lane.source_fps);
    }
    std::filesystem::resize_file(
        lane.payload_path, lane.frame_count * lane.frame_bytes);
  }
  joint_sensor_orchestrator::MockVideoSink sink({
      .frame_bytes = joint_sensor_orchestrator::kVideoFrameBytes,
      .max_lateness_ns = 5'000'000,
      .grid_origin_ns = 0,
      .output_fps = 30,
      .play_start_ns = 1'000,
      .lanes = {{"tx1", 10, 0}, {"tx2", 30, 1}, {"tx4", 10, 2}},
      .failures = {},
  });
  VideoFakeClock clock;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  std::atomic<unsigned> pread_calls{};
  joint_sensor_orchestrator::VideoEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.pread = [&](int, void*, std::size_t,
                           std::uint64_t) -> std::ptrdiff_t {
    ++pread_calls;
    return -1;
  };
  joint_sensor_orchestrator::VideoEngine engine(
      video_config(), std::move(cache), {0, 39'900'000'000ULL}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(6);
  engine.arm(gate);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  gate->release({1'000, 5'000'000});
  const auto result = engine.result();
  const auto health = sink.health();

  CHECK(result.ok);
  CHECK(pread_calls.load() == 0);
  CHECK(health.ok);
  CHECK(health.frames_written == 1197 + 1198 + 1197);
  CHECK(health.bytes_written ==
        static_cast<std::uint64_t>(1197 + 1198 + 1197) *
            joint_sensor_orchestrator::kVideoFrameBytes);
  CHECK(result.lanes[0].source_frames_scheduled == 399);
  CHECK(result.lanes[0].output_frames_written == 1197);
  CHECK(result.lanes[1].source_frames_scheduled == 1198);
  CHECK(result.lanes[1].output_frames_written == 1198);
  CHECK(result.lanes[2].source_frames_scheduled == 399);
  CHECK(result.lanes[2].output_frames_written == 1197);
  for (const auto& lane : result.lanes) {
    CHECK(lane.max_queue_waterline <= 3);
  }
}

TEST_CASE(video_engine_sink_failure_cancels_every_lane_and_joins) {
  VideoFixture fixture;
  VideoFakeClock clock;
  RecordingVideoSink sink;
  sink.fail_lane = "tx2";
  sink.fail_output = 0;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::VideoEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  joint_sensor_orchestrator::VideoEngine engine(
      video_config(), fixture.cache(), {0, 199'999'999}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(6);
  engine.arm(gate);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  gate->release({1'000, 5'000'000});
  const auto result = engine.result();

  CHECK(!result.ok);
  CHECK(result.error == "video sink failed");
  CHECK(lifecycle.failure().component == "video");
  CHECK(lifecycle.failure().reason == "video sink failed");
  CHECK(lifecycle.stop_token()->stop_requested());
  CHECK(sink.stopped.load());
  CHECK(engine.state() ==
        joint_sensor_orchestrator::VideoEngineState::Failed);
}

TEST_CASE(video_engine_rejects_more_than_five_ms_lateness) {
  VideoFixture fixture;
  VideoFakeClock clock;
  clock.late_by_ns = 5'000'001;
  RecordingVideoSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::VideoEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  joint_sensor_orchestrator::VideoEngine engine(
      video_config(), fixture.cache(), {0, 199'999'999}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(6);
  engine.arm(gate);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  gate->release({1'000, 5'000'000});
  const auto result = engine.result();

  CHECK(!result.ok);
  CHECK(result.error.find("视频迟到超过门限") != std::string::npos);
  CHECK(lifecycle.stop_token()->stop_requested());
}

TEST_CASE(video_engine_partial_thread_creation_failure_rolls_back) {
  VideoFixture fixture;
  VideoFakeClock clock;
  RecordingVideoSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  std::atomic<unsigned> calls{};
  joint_sensor_orchestrator::VideoEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.thread_factory =
      [&](std::function<void()> worker) -> std::thread {
    if (++calls == 4) {
      throw std::runtime_error("video thread create failed");
    }
    return std::thread(std::move(worker));
  };
  joint_sensor_orchestrator::VideoEngine engine(
      video_config(), fixture.cache(), {0, 199'999'999}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(6);
  engine.arm(gate);
  CHECK_THROWS(engine.run());

  CHECK(lifecycle.failure().component == "video");
  CHECK(lifecycle.failure().reason == "video thread create failed");
  CHECK(lifecycle.stop_token()->stop_requested());
  CHECK(sink.stopped.load());
  CHECK(engine.state() ==
        joint_sensor_orchestrator::VideoEngineState::Failed);
}

TEST_CASE(video_engine_prepare_failure_stops_sink_and_closes_resources) {
  VideoFixture fixture;
  VideoFakeClock clock;
  RecordingVideoSink sink;
  sink.prepare_fail = true;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::VideoEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  joint_sensor_orchestrator::VideoEngine engine(
      video_config(), fixture.cache(), {0, 199'999'999}, lifecycle,
      std::move(dependencies));

  CHECK_THROWS(engine.prepare());
  CHECK(sink.stopped.load());
  CHECK(lifecycle.failure().reason == "video prepare failed");
}

TEST_CASE(video_engine_long_wait_is_cancelled_in_one_ms_steps) {
  VideoFixture fixture;
  VideoCancellingClock clock;
  RecordingVideoSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  clock.cancellation = lifecycle.stop_token().get();
  joint_sensor_orchestrator::VideoEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  joint_sensor_orchestrator::VideoEngine engine(
      video_config(), fixture.cache(), {0, 199'999'999}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(6);
  engine.arm(gate);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  gate->release({1'000'000'000, 5'000'000});
  const auto result = engine.result();

  CHECK(result.ok);
  CHECK(result.cancelled);
  CHECK(clock.max_step_ns.load() <= 1'000'000);
  CHECK(sink.slots.empty());
}

TEST_CASE(video_engine_prefetch_stops_after_scheduled_frames) {
  VideoFixture fixture;
  auto cached = fixture.cache();
  // 10 Hz 源在窗口内，但 3 次重复的最后一格超出窗口，不会进入输出时间轴。
  // 帧长按页对齐，预读走直接读，避免未对齐读被当成失败。
  for (auto& lane : cached.lanes) {
    if (lane.name == "tx1" || lane.name == "tx4") {
      lane.timestamps_ns = {0, 90'000'000};
    }
    lane.frame_bytes = 4096;
    lane.frame_count = lane.timestamps_ns.size();
    std::ofstream output(lane.payload_path, std::ios::binary | std::ios::trunc);
    const std::vector<char> payload(lane.frame_count * lane.frame_bytes, '\x2a');
    output.write(payload.data(), static_cast<std::streamsize>(payload.size()));
  }
  VideoFakeClock clock;
  RecordingVideoSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  std::atomic<unsigned> pread_calls{};
  joint_sensor_orchestrator::VideoEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.pread = [&](int fd, void* data, std::size_t size,
                           std::uint64_t offset) {
    ++pread_calls;
    return ::pread(fd, data, size, static_cast<off_t>(offset));
  };
  auto config = video_config();
  config.video_backend = joint_sensor_orchestrator::VideoBackend::Xdma;
  config.video_prefetch_ms = 1000;
  joint_sensor_orchestrator::VideoEngine engine(
      config, cached, {0, 100'000'000}, lifecycle, std::move(dependencies));
  engine.prepare();
  auto gate = std::make_shared<joint_sensor_orchestrator::StartGate>(6);
  engine.arm(gate);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  gate->release({1'000'000'000ULL, 5'000'000});

  std::atomic<bool> done{false};
  joint_sensor_orchestrator::VideoEngineResult result;
  std::exception_ptr error;
  std::thread waiter([&] {
    try {
      result = engine.result();
    } catch (...) {
      error = std::current_exception();
    }
    done.store(true);
  });
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!done.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const bool finished_on_time = done.load();
  if (!finished_on_time) lifecycle.request_stop();
  waiter.join();

  CHECK(finished_on_time);
  CHECK(error == nullptr);
  CHECK(result.ok);
  CHECK(!result.cancelled);
  CHECK(sink.slots["tx1"].size() == 3);
  CHECK(sink.slots["tx1"].back().source_index == 0);
  CHECK(sink.slots["tx4"].size() == 3);
  CHECK(sink.slots["tx4"].back().source_index == 0);
  CHECK(sink.slots["tx2"].size() == 2);
  CHECK(pread_calls.load() == 4);
}
