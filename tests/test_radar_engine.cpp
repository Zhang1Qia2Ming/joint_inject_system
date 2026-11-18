#include "joint_sensor_orchestrator/radar_engine.hpp"
#include "joint_sensor_player/packet_time_rewriter.hpp"
#include "test_framework.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <future>
#include <mutex>
#include <vector>

namespace {

class RadarFakeClock final : public joint_sensor_player::Clock {
 public:
  explicit RadarFakeClock(std::uint64_t now) : now_(now) {}

  std::uint64_t now_ns() const override { return now_.load(); }
  bool sleep_until_ns(std::uint64_t deadline_ns) override {
    now_.store(deadline_ns);
    return true;
  }

 private:
  std::atomic<std::uint64_t> now_;
};

class RadarCancellingClock final : public joint_sensor_player::Clock {
 public:
  std::uint64_t now_ns() const override { return now_.load(); }
  bool sleep_until_ns(std::uint64_t deadline_ns) override {
    now_.store(deadline_ns);
    cancellation->request_stop();
    return true;
  }

  joint_sensor_player::CancellationToken* cancellation{};

 private:
  std::atomic<std::uint64_t> now_{};
};

class RadarFakeSink final : public joint_sensor_player::DatagramSink {
 public:
  bool send(const joint_sensor_player::LidarChannelConfig&,
            const std::vector<std::uint8_t>& payload,
            std::uint64_t deadline_ns, std::string& error) override {
    std::lock_guard<std::mutex> lock(mutex);
    if (fail) {
      error = "radar send failed";
      return false;
    }
    deadlines.push_back(deadline_ns);
    packets.push_back(payload);
    return true;
  }

  bool fail{};
  std::mutex mutex;
  std::vector<std::uint64_t> deadlines;
  std::vector<std::vector<std::uint8_t>> packets;
};

class RadarFakePtc final
    : public joint_sensor_orchestrator::RadarPtcService {
 public:
  void start() override { started = true; }
  void stop() noexcept override { stopped = true; }

  bool started{};
  bool stopped{};
};

void radar_write_le32(std::vector<std::uint8_t>& packet, std::size_t offset,
                      std::uint32_t value) {
  std::memcpy(packet.data() + offset, &value, sizeof(value));
}

std::vector<std::uint8_t> radar_packet(std::uint64_t time_us,
                                       std::uint32_t sequence) {
  std::vector<std::uint8_t> packet(1118);
  packet[0] = 0xEE;
  packet[1] = 0xFF;
  packet[2] = 0x04;
  packet[3] = 0x03;
  packet[6] = 128;
  packet[7] = 2;
  packet[11] = 1;
  radar_write_le32(packet, 1066,
                   static_cast<std::uint32_t>(time_us % 1000000));
  const auto seconds = static_cast<std::uint32_t>(time_us / 1000000);
  packet[1074] = static_cast<std::uint8_t>(seconds >> 24U);
  packet[1075] = static_cast<std::uint8_t>(seconds >> 16U);
  packet[1076] = static_cast<std::uint8_t>(seconds >> 8U);
  packet[1077] = static_cast<std::uint8_t>(seconds);
  radar_write_le32(packet, 1078, sequence);
  radar_write_le32(
      packet, 1082,
      joint_sensor_player::lidar_crc32(packet.data() + 1046, 36));
  return packet;
}

joint_sensor_player::McapCatalog radar_catalog(
    const joint_sensor_player::LidarChannelConfig& channel) {
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[channel.topic] = {
      {100'000'000, 100'000'000, 1, {},
       {radar_packet(1'000'000, 1)}},
      {200'000'000, 200'000'000, 2, {},
       {radar_packet(1'100'000, 2)}}};
  return catalog;
}

joint_sensor_player::PlayerConfig radar_config() {
  auto channel = joint_sensor_player::default_lidar_channels()[0];
  joint_sensor_player::PlayerConfig config;
  config.channels = {channel};
  config.start_delay_ms = 500;
  config.loop_count = 1;
  config.ptc_enabled = false;
  return config;
}

}  // namespace

TEST_CASE(radar_engine_stays_ready_until_armed_and_uses_shared_anchors) {
  auto config = radar_config();
  auto catalog = radar_catalog(config.channels.front());
  RadarFakeClock clock(500);
  RadarFakeSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::RadarEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.scan = [catalog](const auto&, const auto&) { return catalog; };
  joint_sensor_orchestrator::RadarEngine engine(
      config, {100'000'000, 200'000'000}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  CHECK(engine.state() ==
        joint_sensor_orchestrator::RadarEngineState::Ready);
  CHECK(sink.deadlines.empty());

  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(1);
  engine.arm(gate);
  CHECK(engine.state() ==
        joint_sensor_orchestrator::RadarEngineState::Armed);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  CHECK(sink.deadlines.empty());
  gate->release({10'000, 2'000'000});
  const auto result = engine.result();

  CHECK(result.playback.ok);
  CHECK(result.playback.play_start_ns == 10'000);
  CHECK(result.utc_anchor_us == 2'000'000);
  CHECK(sink.deadlines.front() == 10'000);
  const auto rewritten =
      joint_sensor_player::inspect_lidar_packet(sink.packets.front());
  CHECK(rewritten.sensor_time_us == 2'000'000);
  CHECK(engine.state() ==
        joint_sensor_orchestrator::RadarEngineState::Stopped);
}

TEST_CASE(radar_engine_filters_and_schedules_by_bag_log_time) {
  auto config = radar_config();
  const auto& channel = config.channels.front();
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[channel.topic] = {
      {2'200'000'000ULL, 300'000'000ULL, 3, {},
       {radar_packet(1'200'000, 3)}},
      {2'000'000'000ULL, 100'000'000ULL, 1, {},
       {radar_packet(1'000'000, 1)}},
      {2'100'000'000ULL, 200'000'000ULL, 2, {},
       {radar_packet(1'100'000, 2)}},
  };
  RadarFakeClock clock(500);
  RadarFakeSink sink;
  std::vector<joint_sensor_player::FrameStat> observed;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::RadarEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.scan = [catalog](const auto&, const auto&) { return catalog; };
  dependencies.frame_observer =
      [&observed](const joint_sensor_player::FrameStat& stat) {
        observed.push_back(stat);
      };
  joint_sensor_orchestrator::RadarEngine engine(
      config, {150'000'000ULL, 300'000'000ULL}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(1);
  engine.arm(gate);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  gate->release({10'000, 2'000'000});
  const auto result = engine.result();

  CHECK(result.playback.ok);
  CHECK(result.playback.frames_sent == 2);
  CHECK(sink.deadlines.size() == 2);
  CHECK(sink.deadlines[0] == 50'010'000ULL);
  CHECK(sink.deadlines[1] == 150'010'000ULL);
  CHECK(observed.size() == 2);
  CHECK(observed[0].source_timestamp_ns == 200'000'000ULL);
  CHECK(observed[1].source_timestamp_ns == 300'000'000ULL);
}

TEST_CASE(radar_engine_rounds_cycle_up_to_whole_microsecond) {
  auto config = radar_config();
  const auto& channel = config.channels.front();
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[channel.topic] = {
      {2'000'000'000ULL, 100'000'123ULL, 1, {},
       {radar_packet(1'000'000, 1)}},
      {2'100'000'000ULL, 200'000'456ULL, 2, {},
       {radar_packet(1'200'000, 2)}},
  };
  RadarFakeClock clock(500);
  RadarFakeSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::RadarEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.scan = [catalog](const auto&, const auto&) { return catalog; };
  joint_sensor_orchestrator::RadarEngine engine(
      config, {100'000'123ULL, 200'000'456ULL}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(1);
  engine.arm(gate);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  gate->release({10'000, 2'000'000});
  const auto result = engine.result();

  CHECK(result.playback.ok);
  CHECK(result.playback.frames_sent == 2);
  CHECK(sink.deadlines.size() == 2);
  CHECK(sink.deadlines[1] == 100'010'333ULL);
}

TEST_CASE(radar_engine_single_pass_allows_payload_span_beyond_cycle_period) {
  auto config = radar_config();
  const auto& channel = config.channels.front();
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[channel.topic] = {
      {2'000'000'000ULL, 100'000'000ULL, 1, {},
       {radar_packet(1'000'000, 1)}},
      {2'100'000'000ULL, 200'000'000ULL, 2, {},
       {radar_packet(1'200'001, 2)}},
  };
  RadarFakeClock clock(500);
  RadarFakeSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::RadarEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.scan = [catalog](const auto&, const auto&) { return catalog; };
  joint_sensor_orchestrator::RadarEngine engine(
      config, {100'000'000ULL, 200'000'000ULL}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(1);
  engine.arm(gate);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  gate->release({10'000, 2'000'000});
  const auto result = engine.result();

  CHECK(result.playback.ok);
  CHECK(result.playback.frames_sent == 2);
  CHECK(sink.packets.size() == 2);
  CHECK(joint_sensor_player::inspect_lidar_packet(sink.packets[1])
            .sensor_time_us == 2'200'001ULL);
}

TEST_CASE(radar_engine_consumes_prevalidated_catalog_without_rescanning) {
  auto config = radar_config();
  auto catalog = radar_catalog(config.channels.front());
  RadarFakeClock clock(500);
  RadarFakeSink sink;
  int scan_calls = 0;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::RadarEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.prevalidated_catalog = std::move(catalog);
  dependencies.scan = [&scan_calls](const auto&, const auto&) {
    ++scan_calls;
    throw std::runtime_error("不应重复扫描");
    return joint_sensor_player::McapCatalog{};
  };
  joint_sensor_orchestrator::RadarEngine engine(
      config, {100'000'000, 200'000'000}, lifecycle,
      std::move(dependencies));

  engine.prepare();

  CHECK(scan_calls == 0);
  CHECK(engine.state() ==
        joint_sensor_orchestrator::RadarEngineState::Ready);
}

TEST_CASE(radar_engine_send_failure_preserves_root_and_stops_ptc) {
  auto config = radar_config();
  config.ptc_enabled = true;
  auto catalog = radar_catalog(config.channels.front());
  RadarFakeClock clock(500);
  RadarFakeSink sink;
  sink.fail = true;
  auto ptc = std::make_unique<RadarFakePtc>();
  auto* ptc_observer = ptc.get();
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::RadarEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.ptc = std::move(ptc);
  dependencies.scan = [catalog](const auto&, const auto&) { return catalog; };
  joint_sensor_orchestrator::RadarEngine engine(
      config, {100'000'000, 200'000'000}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  CHECK(ptc_observer->started);
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(1);
  engine.arm(gate);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  gate->release({10'000, 2'000'000});
  const auto result = engine.result();

  CHECK(!result.playback.ok);
  CHECK(lifecycle.failure().component == "radar");
  CHECK(lifecycle.failure().reason == "radar send failed");
  CHECK(lifecycle.stop_token()->stop_requested());
  CHECK(ptc_observer->stopped);
}

TEST_CASE(radar_engine_obeys_shared_cancellation_before_running) {
  auto config = radar_config();
  auto catalog = radar_catalog(config.channels.front());
  RadarFakeClock clock(500);
  RadarFakeSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::RadarEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.scan = [catalog](const auto&, const auto&) { return catalog; };
  joint_sensor_orchestrator::RadarEngine engine(
      config, {100'000'000, 200'000'000}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(1);
  engine.arm(gate);
  lifecycle.request_stop();
  engine.run();
  const auto result = engine.result();

  CHECK(result.playback.ok);
  CHECK(result.playback.cancelled);
  CHECK(sink.deadlines.empty());
}

TEST_CASE(radar_engine_post_gate_setup_failure_preserves_root_and_cancels) {
  auto config = radar_config();
  auto catalog = radar_catalog(config.channels.front());
  catalog.frames[config.channels.front().topic][0].packets[0] = {0};
  RadarFakeClock clock(500);
  RadarFakeSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::RadarEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.scan = [catalog](const auto&, const auto&) { return catalog; };
  joint_sensor_orchestrator::RadarEngine engine(
      config, {100'000'000, 200'000'000}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(1);
  engine.arm(gate);
  engine.run();
  CHECK(gate->wait_until_ready(std::chrono::milliseconds(100)));
  gate->release({10'000, 2'000'000});
  const auto result = engine.result();
  CHECK(!result.playback.ok);
  CHECK(lifecycle.failure().component == "radar");
  CHECK(!lifecycle.failure().reason.empty());
  CHECK(lifecycle.stop_token()->stop_requested());
}

TEST_CASE(radar_engine_thread_creation_failure_rolls_back_everything) {
  auto config = radar_config();
  config.ptc_enabled = true;
  auto catalog = radar_catalog(config.channels.front());
  RadarFakeClock clock(500);
  RadarFakeSink sink;
  auto ptc = std::make_unique<RadarFakePtc>();
  auto* ptc_observer = ptc.get();
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::RadarEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.ptc = std::move(ptc);
  dependencies.scan = [catalog](const auto&, const auto&) { return catalog; };
  dependencies.thread_factory = [](std::function<void()>) -> std::thread {
    throw std::runtime_error("radar thread create failed");
  };
  joint_sensor_orchestrator::RadarEngine engine(
      config, {100'000'000, 200'000'000}, lifecycle,
      std::move(dependencies));

  engine.prepare();
  auto gate =
      std::make_shared<joint_sensor_orchestrator::StartGate>(1);
  engine.arm(gate);
  CHECK_THROWS(engine.run());

  CHECK(lifecycle.failure().component == "radar");
  CHECK(lifecycle.failure().reason == "radar thread create failed");
  CHECK(lifecycle.stop_token()->stop_requested());
  CHECK(ptc_observer->stopped);
  CHECK(engine.state() ==
        joint_sensor_orchestrator::RadarEngineState::Failed);
}

TEST_CASE(streaming_radar_globally_interleaves_deadlines_and_rewrites_utc) {
  auto config = radar_config();
  auto second = joint_sensor_player::default_lidar_channels()[1];
  config.channels.push_back(second);
  config.channels[0].packet_interval_ns = 20;
  config.channels[1].packet_interval_ns = 10;
  joint_sensor_player::LightweightMcapCatalog catalog;
  for (const auto& channel : config.channels) {
    catalog.frames[channel.topic] = {
        {.source_timestamp_ns = 100,
         .bag_log_time_ns = 100,
         .packet_count = 2,
         .first_packet_time_us = 1'000'000,
         .last_packet_time_us = 1'000'100,
         .first_packet_sequence = 1,
         .last_packet_sequence = 2,
         .protocol = 0}};
  }
  RadarFakeClock clock(0);
  RadarFakeSink sink;
  joint_sensor_player::CancellationToken cancellation;
  const auto stream =
      [&](const joint_sensor_player::McapFrameObserver& observer) {
        for (const auto& channel : config.channels) {
          joint_sensor_player::LidarFrame frame{
              100, 100, 1, {},
              {radar_packet(1'000'000, 1),
               radar_packet(1'000'100, 2)}};
          CHECK(observer(channel.topic, std::move(frame)));
        }
        return joint_sensor_player::McapStreamStats{2, 2};
      };

  const auto result = joint_sensor_orchestrator::play_streaming_radar(
      config, catalog, {100, 100}, clock, sink, cancellation,
      2'000'000, 1'000, stream, {},
      {.packet_queue_capacity = 8});

  CHECK(result.ok);
  CHECK(result.frames_sent == 2);
  CHECK(result.packets_sent == 4);
  CHECK(result.max_buffered_packets <= 8);
  auto deadlines = sink.deadlines;
  std::sort(deadlines.begin(), deadlines.end());
  CHECK(deadlines ==
        std::vector<std::uint64_t>({1'000, 1'000, 1'010, 1'020}));
  int rewritten_origin = 0;
  for (std::size_t index = 0; index < sink.packets.size(); ++index) {
    const auto sensor_time =
        joint_sensor_player::inspect_lidar_packet(sink.packets[index])
            .sensor_time_us;
    if (sink.deadlines[index] == 1'000) {
      CHECK(sensor_time == 2'000'000);
      ++rewritten_origin;
    } else {
      CHECK(sensor_time == 2'000'100);
    }
  }
  CHECK(rewritten_origin == 2);
}

TEST_CASE(streaming_radar_cancellation_stops_bounded_producer) {
  auto config = radar_config();
  joint_sensor_player::LightweightMcapCatalog catalog;
  catalog.frames[config.channels.front().topic] = {
      {.source_timestamp_ns = 100,
       .bag_log_time_ns = 100,
       .packet_count = 1,
       .first_packet_time_us = 1'000'000,
       .last_packet_time_us = 1'000'000,
       .first_packet_sequence = 1,
       .last_packet_sequence = 1,
       .protocol = 0}};
  RadarFakeClock clock(0);
  RadarFakeSink sink;
  joint_sensor_player::CancellationToken cancellation;
  cancellation.request_stop();
  const auto result = joint_sensor_orchestrator::play_streaming_radar(
      config, catalog, {100, 100}, clock, sink, cancellation,
      2'000'000, 1'000,
      [](const joint_sensor_player::McapFrameObserver&) {
        throw std::runtime_error("取消后不应启动生产者");
        return joint_sensor_player::McapStreamStats{};
      });
  CHECK(result.ok);
  CHECK(result.cancelled);
  CHECK(sink.packets.empty());
}

TEST_CASE(radar_engine_prepares_lightweight_catalog_without_full_scan) {
  auto config = radar_config();
  joint_sensor_player::LightweightMcapCatalog catalog;
  catalog.required_reorder_frames = 81;
  catalog.frames[config.channels.front().topic] = {
      {.source_timestamp_ns = 100,
       .bag_log_time_ns = 100,
       .packet_count = 1,
       .first_packet_time_us = 1'000'000,
       .last_packet_time_us = 1'000'000,
       .first_packet_sequence = 1,
       .last_packet_sequence = 1,
       .protocol = 0}};
  RadarFakeClock clock(0);
  RadarFakeSink sink;
  int full_scans = 0;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::RadarEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.prevalidated_lightweight_catalog = std::move(catalog);
  dependencies.scan = [&](const auto&, const auto&) {
    ++full_scans;
    return joint_sensor_player::McapCatalog{};
  };
  joint_sensor_orchestrator::RadarEngine engine(
      config, {100, 100}, lifecycle, std::move(dependencies));
  engine.prepare();
  CHECK(full_scans == 0);
  CHECK(engine.state() ==
        joint_sensor_orchestrator::RadarEngineState::Ready);
}

TEST_CASE(radar_engine_rejects_129_reorder_frames_with_default_limit) {
  auto config = radar_config();
  joint_sensor_player::LightweightMcapCatalog catalog;
  catalog.required_reorder_frames = 129;
  catalog.frames[config.channels.front().topic] = {
      {.source_timestamp_ns = 100,
       .bag_log_time_ns = 100,
       .packet_count = 1,
       .first_packet_time_us = 1'000'000,
       .last_packet_time_us = 1'000'000,
       .first_packet_sequence = 1,
       .last_packet_sequence = 1,
       .protocol = 0}};
  RadarFakeClock clock(0);
  RadarFakeSink sink;
  joint_sensor_orchestrator::LifecycleManager lifecycle;
  joint_sensor_orchestrator::RadarEngineDependencies dependencies;
  dependencies.clock = &clock;
  dependencies.sink = &sink;
  dependencies.prevalidated_lightweight_catalog = std::move(catalog);
  joint_sensor_orchestrator::RadarEngine engine(
      config, {100, 100}, lifecycle, std::move(dependencies));
  CHECK_THROWS(engine.prepare());
  CHECK(lifecycle.failure().reason.find("limit=128") !=
        std::string::npos);
}

TEST_CASE(
    streaming_radar_releases_completed_bag_time_before_next_stream_frame) {
  auto config = radar_config();
  config.channels.front().packet_interval_ns = 100;
  config.max_lateness_us = 20'000;
  joint_sensor_player::LightweightMcapCatalog catalog;
  catalog.frames[config.channels.front().topic] = {
      {.source_timestamp_ns = 100,
       .bag_log_time_ns = 100,
       .packet_count = 1,
       .first_packet_time_us = 1'000'000,
       .last_packet_time_us = 1'000'000,
       .first_packet_sequence = 1,
       .last_packet_sequence = 1,
       .protocol = 0},
      {.source_timestamp_ns = 50'000'000,
       .bag_log_time_ns = 50'000'000,
       .packet_count = 1,
       .first_packet_time_us = 1'050'000,
       .last_packet_time_us = 1'050'000,
       .first_packet_sequence = 2,
       .last_packet_sequence = 2,
       .protocol = 0},
  };
  RadarFakeClock clock(0);
  RadarFakeSink sink;
  std::promise<void> first_packet_sent;
  auto first_sent = first_packet_sent.get_future();
  std::atomic<bool> signaled{false};
  joint_sensor_player::CancellationToken cancellation;
  const auto result = joint_sensor_orchestrator::play_streaming_radar(
      config, catalog, {100, 50'000'000}, clock, sink, cancellation,
      2'000'000, 1'000,
      [&](const joint_sensor_player::McapFrameObserver& observer) {
        CHECK(observer(
            config.channels.front().topic,
            {100, 100, 1, {}, {radar_packet(1'000'000, 1)}}));
        const auto status =
            first_sent.wait_for(std::chrono::milliseconds(500));
        CHECK(status == std::future_status::ready);
        CHECK(observer(
            config.channels.front().topic,
            {50'000'000, 50'000'000, 2, {},
             {radar_packet(1'050'000, 2)}}));
        return joint_sensor_player::McapStreamStats{2, 2};
      },
      [&](const joint_sensor_player::FrameStat&) {
        if (!signaled.exchange(true)) {
          first_packet_sent.set_value();
        }
      },
      {.packet_queue_capacity = 4});
  CHECK(result.ok);
  CHECK(sink.deadlines ==
        std::vector<std::uint64_t>({1'000, 50'000'900}));
}

TEST_CASE(streaming_radar_holds_old_late_packet_until_new_early_packet_arrives) {
  auto config = radar_config();
  config.channels.front().packet_interval_ns = 200;
  joint_sensor_player::LightweightMcapCatalog catalog;
  catalog.frames[config.channels.front().topic] = {
      {.source_timestamp_ns = 100,
       .bag_log_time_ns = 100,
       .packet_count = 2,
       .first_packet_time_us = 1'000'000,
       .last_packet_time_us = 1'000'100,
       .first_packet_sequence = 1,
       .last_packet_sequence = 2,
       .protocol = 0},
      {.source_timestamp_ns = 110,
       .bag_log_time_ns = 110,
       .packet_count = 1,
       .first_packet_time_us = 1'000'200,
       .last_packet_time_us = 1'000'200,
       .first_packet_sequence = 3,
       .last_packet_sequence = 3,
       .protocol = 0},
  };
  auto bound_config = config;
  auto negative_offset_channel =
      joint_sensor_player::default_lidar_channels()[1];
  negative_offset_channel.time_offset_us = -1;
  bound_config.channels.push_back(negative_offset_channel);
  const auto future_lower_bound =
      joint_sensor_orchestrator::earliest_possible_frame_deadline_ns(
          bound_config, 100, 110, 2'000);
  CHECK(future_lower_bound == 1'010);
  CHECK(!joint_sensor_orchestrator::packet_deadline_is_safe(
      2'200, future_lower_bound));

  RadarFakeClock clock(0);
  RadarFakeSink sink;
  joint_sensor_player::CancellationToken cancellation;
  const auto result = joint_sensor_orchestrator::play_streaming_radar(
      config, catalog, {100, 110}, clock, sink, cancellation,
      2'000'000, 1'000,
      [&](const joint_sensor_player::McapFrameObserver& observer) {
        CHECK(observer(
            config.channels.front().topic,
            {100, 100, 1, {},
             {radar_packet(1'000'000, 1),
              radar_packet(1'000'100, 2)}}));
        CHECK(observer(
            config.channels.front().topic,
            {110, 110, 2, {}, {radar_packet(1'000'200, 3)}}));
        return joint_sensor_player::McapStreamStats{2, 2};
      },
      {}, {.packet_queue_capacity = 2});

  CHECK(result.ok);
  CHECK(sink.deadlines ==
        std::vector<std::uint64_t>({1'000, 1'010, 1'200}));
}

TEST_CASE(streaming_radar_preflight_counts_cross_frame_deadline_overlap) {
  auto config = radar_config();
  config.channels.front().packet_interval_ns = 100;
  joint_sensor_player::LightweightMcapCatalog catalog;
  catalog.required_reorder_frames = 81;
  catalog.frames[config.channels.front().topic] = {
      {.source_timestamp_ns = 100,
       .bag_log_time_ns = 100,
       .packet_count = 5},
      {.source_timestamp_ns = 150,
       .bag_log_time_ns = 150,
       .packet_count = 5},
  };
  const auto preflight =
      joint_sensor_orchestrator::analyze_streaming_radar(
          config, catalog, {100, 150});
  CHECK(preflight.required_frame_reorder == 81);
  CHECK(preflight.required_packet_buffer == 9);
}

TEST_CASE(streaming_radar_same_deadline_preserves_frame_then_packet_order) {
  auto config = radar_config();
  config.channels.front().packet_interval_ns = 10;
  joint_sensor_player::LightweightMcapCatalog catalog;
  catalog.frames[config.channels.front().topic] = {
      {.source_timestamp_ns = 100,
       .bag_log_time_ns = 100,
       .packet_count = 2,
       .first_packet_time_us = 1'000'000,
       .last_packet_time_us = 1'000'001,
       .first_packet_sequence = 1,
       .last_packet_sequence = 2,
       .protocol = 0},
      {.source_timestamp_ns = 110,
       .bag_log_time_ns = 110,
       .packet_count = 1,
       .first_packet_time_us = 1'000'002,
       .last_packet_time_us = 1'000'002,
       .first_packet_sequence = 3,
       .last_packet_sequence = 3,
       .protocol = 0},
  };
  RadarFakeClock clock(0);
  RadarFakeSink sink;
  joint_sensor_player::CancellationToken cancellation;
  const auto result = joint_sensor_orchestrator::play_streaming_radar(
      config, catalog, {100, 110}, clock, sink, cancellation,
      2'000'000, 1'000,
      [&](const joint_sensor_player::McapFrameObserver& observer) {
        CHECK(observer(
            config.channels.front().topic,
            {100, 100, 1, {},
             {radar_packet(1'000'000, 1),
              radar_packet(1'000'001, 2)}}));
        CHECK(observer(
            config.channels.front().topic,
            {110, 110, 2, {}, {radar_packet(1'000'002, 3)}}));
        return joint_sensor_player::McapStreamStats{2, 2};
      },
      {}, {.packet_queue_capacity = 3});

  CHECK(result.ok);
  CHECK(sink.deadlines ==
        std::vector<std::uint64_t>({1'000, 1'010, 1'010}));
  std::vector<std::uint32_t> sequences;
  for (const auto& packet : sink.packets) {
    sequences.push_back(
        joint_sensor_player::inspect_lidar_packet(packet).sequence);
  }
  CHECK(sequences == std::vector<std::uint32_t>({1, 2, 3}));
}

TEST_CASE(streaming_radar_accepts_exactly_8192_cross_frame_packet_heap) {
  auto config = radar_config();
  config.channels.front().packet_interval_ns = 1;
  constexpr std::uint32_t first_count = 4097;
  constexpr std::uint32_t second_count = 4096;
  joint_sensor_player::LightweightMcapCatalog catalog;
  catalog.frames[config.channels.front().topic] = {
      {.source_timestamp_ns = 100,
       .bag_log_time_ns = 100,
       .packet_count = first_count,
       .first_packet_time_us = 1'000'000,
       .last_packet_time_us = 1'000'000 + first_count - 1,
       .first_packet_sequence = 1,
       .last_packet_sequence = first_count,
       .protocol = 0},
      {.source_timestamp_ns = 101,
       .bag_log_time_ns = 101,
       .packet_count = second_count,
       .first_packet_time_us = 1'000'000 + first_count,
       .last_packet_time_us =
           1'000'000 + first_count + second_count - 1,
       .first_packet_sequence = first_count + 1,
       .last_packet_sequence = first_count + second_count,
       .protocol = 0}};
  CHECK(joint_sensor_orchestrator::analyze_streaming_radar(
            config, catalog, {100, 101})
            .required_packet_buffer == 8192);
  RadarFakeClock clock(0);
  RadarFakeSink sink;
  joint_sensor_player::CancellationToken cancellation;
  const auto result = joint_sensor_orchestrator::play_streaming_radar(
      config, catalog, {100, 101}, clock, sink, cancellation,
      2'000'000, 1'000,
      [&](const joint_sensor_player::McapFrameObserver& observer) {
        std::vector<std::vector<std::uint8_t>> first;
        first.reserve(first_count);
        for (std::uint32_t index = 0; index < first_count; ++index) {
          first.push_back(radar_packet(1'000'000 + index, index + 1));
        }
        CHECK(observer(config.channels.front().topic,
                       {100, 100, 1, {}, std::move(first)}));
        std::vector<std::vector<std::uint8_t>> second;
        second.reserve(second_count);
        for (std::uint32_t index = 0; index < second_count; ++index) {
          second.push_back(radar_packet(
              1'000'000 + first_count + index,
              first_count + index + 1));
        }
        CHECK(observer(config.channels.front().topic,
                       {101, 101, 2, {}, std::move(second)}));
        return joint_sensor_player::McapStreamStats{2, 2};
      });
  CHECK(result.ok);
  CHECK(result.packets_sent == first_count + second_count);
  CHECK(result.max_buffered_packets == 8192);
}

TEST_CASE(streaming_radar_rejects_8193_cross_frame_heap_before_producer) {
  auto config = radar_config();
  config.channels.front().packet_interval_ns = 1;
  constexpr std::uint32_t packet_count = 4097;
  joint_sensor_player::LightweightMcapCatalog catalog;
  catalog.frames[config.channels.front().topic] = {
      {.source_timestamp_ns = 100,
       .bag_log_time_ns = 100,
       .packet_count = packet_count,
       .first_packet_time_us = 1'000'000,
       .last_packet_time_us = 1'000'000 + packet_count - 1,
       .first_packet_sequence = 1,
       .last_packet_sequence = packet_count,
       .protocol = 0},
      {.source_timestamp_ns = 101,
       .bag_log_time_ns = 101,
       .packet_count = packet_count,
       .first_packet_time_us = 1'000'000 + packet_count,
       .last_packet_time_us = 1'000'000 + packet_count * 2 - 1,
       .first_packet_sequence = packet_count + 1,
       .last_packet_sequence = packet_count * 2,
       .protocol = 0}};
  CHECK(joint_sensor_orchestrator::analyze_streaming_radar(
            config, catalog, {100, 101})
            .required_packet_buffer == 8193);
  RadarFakeClock clock(0);
  RadarFakeSink sink;
  joint_sensor_player::CancellationToken cancellation;
  std::atomic<bool> producer_started{};
  auto run = std::async(std::launch::async, [&] {
    return joint_sensor_orchestrator::play_streaming_radar(
        config, catalog, {100, 101}, clock, sink, cancellation,
        2'000'000, 1'000,
        [&](const joint_sensor_player::McapFrameObserver&) {
          producer_started = true;
          return joint_sensor_player::McapStreamStats{};
        });
  });
  CHECK(run.wait_for(std::chrono::milliseconds(100)) ==
        std::future_status::ready);
  const auto result = run.get();
  CHECK(!result.ok);
  CHECK(!producer_started.load());
  CHECK(result.error.find("8192") != std::string::npos);
}

TEST_CASE(streaming_radar_runtime_cancellation_converges_without_deadlock) {
  auto config = radar_config();
  joint_sensor_player::LightweightMcapCatalog catalog;
  catalog.frames[config.channels.front().topic] = {
      {.source_timestamp_ns = 100,
       .bag_log_time_ns = 100,
       .packet_count = 2,
       .first_packet_time_us = 1'000'000,
       .last_packet_time_us = 1'000'001,
       .first_packet_sequence = 1,
       .last_packet_sequence = 2,
       .protocol = 0}};
  joint_sensor_player::CancellationToken cancellation;
  RadarCancellingClock clock;
  clock.cancellation = &cancellation;
  RadarFakeSink sink;
  auto run = std::async(std::launch::async, [&] {
    return joint_sensor_orchestrator::play_streaming_radar(
        config, catalog, {100, 100}, clock, sink, cancellation,
        2'000'000, 1'000'000,
        [&](const joint_sensor_player::McapFrameObserver& observer) {
          CHECK(observer(
              config.channels.front().topic,
              {100, 100, 1, {},
               {radar_packet(1'000'000, 1),
                radar_packet(1'000'001, 2)}}));
          return joint_sensor_player::McapStreamStats{1, 1};
        },
        {}, {.packet_queue_capacity = 2});
  });
  CHECK(run.wait_for(std::chrono::milliseconds(100)) ==
        std::future_status::ready);
  const auto result = run.get();
  CHECK(result.ok);
  CHECK(result.cancelled);
  CHECK(sink.packets.empty());
}

TEST_CASE(streaming_radar_sends_other_channel_while_one_send_blocks) {
  auto config = radar_config();
  config.channels.push_back(
      joint_sensor_player::default_lidar_channels()[1]);
  config.max_lateness_us = 1'000'000;
  joint_sensor_player::LightweightMcapCatalog catalog;
  for (const auto& channel : config.channels) {
    catalog.frames[channel.topic] = {
        {.source_timestamp_ns = 100,
         .bag_log_time_ns = 100,
         .packet_count = 1,
         .first_packet_time_us = 1'000'000,
         .last_packet_time_us = 1'000'000,
         .first_packet_sequence = 1,
         .last_packet_sequence = 1,
         .protocol = 0}};
  }

  class BlockingSink final : public joint_sensor_player::DatagramSink {
   public:
    bool send(const joint_sensor_player::LidarChannelConfig&,
              const std::vector<std::uint8_t>&,
              std::uint64_t, std::string&) override {
      const int mine = ++entered;
      if (mine == 1) {
        std::unique_lock<std::mutex> lock(mutex);
        const auto other_arrived =
            cv.wait_for(lock, std::chrono::milliseconds(300),
                        [&] { return completed.load() >= 1; });
        CHECK(other_arrived);
        return true;
      }
      completed.store(1);
      cv.notify_all();
      return true;
    }

    std::mutex mutex;
    std::condition_variable cv;
    std::atomic<int> entered{0};
    std::atomic<int> completed{0};
  };

  RadarFakeClock clock(0);
  BlockingSink sink;
  joint_sensor_player::CancellationToken cancellation;
  const auto result = joint_sensor_orchestrator::play_streaming_radar(
      config, catalog, {100, 100}, clock, sink, cancellation,
      2'000'000, 1'000,
      [&](const joint_sensor_player::McapFrameObserver& observer) {
        for (const auto& channel : config.channels) {
          CHECK(observer(channel.topic,
                         {100, 100, 1, {}, {radar_packet(1'000'000, 1)}}));
        }
        return joint_sensor_player::McapStreamStats{2, 2};
      },
      {}, {.packet_queue_capacity = 8});
  CHECK(result.ok);
  CHECK(result.packets_sent == 2);
}

TEST_CASE(streaming_radar_cancellation_unblocks_full_lane_queue) {
  auto config = radar_config();
  config.max_lateness_us = 1'000'000;
  constexpr std::uint32_t packet_count = 300;
  std::vector<std::vector<std::uint8_t>> packets;
  packets.reserve(packet_count);
  for (std::uint32_t seq = 1; seq <= packet_count; ++seq) {
    packets.push_back(radar_packet(1'000'000 + seq, seq));
  }
  joint_sensor_player::LightweightMcapCatalog catalog;
  catalog.frames[config.channels.front().topic] = {
      {.source_timestamp_ns = 100,
       .bag_log_time_ns = 100,
       .packet_count = packet_count,
       .first_packet_time_us = 1'000'001,
       .last_packet_time_us = 1'000'000 + packet_count,
       .first_packet_sequence = 1,
       .last_packet_sequence = packet_count,
       .protocol = 0}};

  class CancelOnWaitClock final : public joint_sensor_player::Clock {
   public:
    std::uint64_t now_ns() const override { return 0; }
    bool sleep_until_ns(std::uint64_t) override {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      cancellation->request_stop();
      return true;
    }
    joint_sensor_player::CancellationToken* cancellation{};
  };

  struct RunState {
    joint_sensor_player::CancellationToken cancellation;
    CancelOnWaitClock clock;
    RadarFakeSink sink;
    std::promise<joint_sensor_player::PlaybackResult> done_promise;
  };
  auto state = std::make_shared<RunState>();
  state->clock.cancellation = &state->cancellation;
  auto done_future = state->done_promise.get_future();
  std::thread runner([state, config, catalog,
                      packets = std::move(packets)]() mutable {
    state->done_promise.set_value(
        joint_sensor_orchestrator::play_streaming_radar(
            config, catalog, {100, 100}, state->clock, state->sink,
            state->cancellation, 2'000'000, 1'000'000'000ULL,
            [config, &packets](
                const joint_sensor_player::McapFrameObserver& observer) {
              if (!observer(config.channels.front().topic,
                            {100, 100, 1, {}, std::move(packets)})) {
                return joint_sensor_player::McapStreamStats{0, 0};
              }
              return joint_sensor_player::McapStreamStats{1, packet_count};
            },
            {}, {.packet_queue_capacity = 512}));
  });
  const auto status = done_future.wait_for(std::chrono::milliseconds(500));
  if (status != std::future_status::ready) {
    runner.detach();
    CHECK(status == std::future_status::ready);
  }
  runner.join();
  const auto result = done_future.get();
  CHECK(result.ok);
  CHECK(result.cancelled);
  CHECK(state->sink.packets.empty());
}
