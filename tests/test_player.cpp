#include "joint_sensor_player/player.hpp"
#include "joint_sensor_player/packet_time_rewriter.hpp"
#include "test_framework.hpp"

#include <csignal>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace {

class FakeClock final : public joint_sensor_player::Clock {
 public:
  explicit FakeClock(std::uint64_t now) : now_(now) {}
  std::uint64_t now_ns() const override { return now_; }
  bool sleep_until_ns(std::uint64_t deadline) override {
    sleep_deadlines.push_back(deadline);
    if (deadline > now_) now_ = deadline + late_by_ns;
    if (cancel_on_sleep != nullptr) cancel_on_sleep->request_stop();
    return true;
  }

  std::uint64_t late_by_ns{};
  joint_sensor_player::CancellationToken* cancel_on_sleep{};
  std::vector<std::uint64_t> sleep_deadlines;

 private:
  std::uint64_t now_;
};

struct Sent {
  int slot;
  std::uint8_t value;
  std::uint64_t deadline;
  std::vector<std::uint8_t> payload;
};

class FakeSink final : public joint_sensor_player::DatagramSink {
 public:
  bool send(const joint_sensor_player::LidarChannelConfig& channel,
            const std::vector<std::uint8_t>& payload,
            std::uint64_t deadline_ns, std::string& error) override {
    if (fail_after >= 0 && static_cast<int>(sent.size()) >= fail_after) {
      error = "injected failure";
      return false;
    }
    sent.push_back({channel.slot, payload.front(), deadline_ns, payload});
    if (cancel_token != nullptr && cancel_after > 0 &&
        static_cast<int>(sent.size()) >= cancel_after) {
      cancel_token->request_stop();
    }
    return true;
  }

  int fail_after{-1};
  int cancel_after{-1};
  joint_sensor_player::CancellationToken* cancel_token{};
  std::vector<Sent> sent;
};

joint_sensor_player::LidarFrame make_frame(
    std::uint64_t timestamp, std::initializer_list<std::uint8_t> packets) {
  joint_sensor_player::LidarFrame frame;
  frame.source_timestamp_ns = timestamp;
  for (const auto value : packets) frame.packets.push_back({value});
  return frame;
}

void write_le32(std::vector<std::uint8_t>& packet, std::size_t offset,
                std::uint32_t value) {
  std::memcpy(packet.data() + offset, &value, sizeof(value));
}

std::vector<std::uint8_t> main_packet(std::uint64_t time_us,
                                      std::uint32_t sequence) {
  std::vector<std::uint8_t> packet(1118);
  packet[0] = 0xEE;
  packet[1] = 0xFF;
  packet[2] = 0x04;
  packet[3] = 0x03;
  packet[6] = 128;
  packet[7] = 2;
  packet[11] = 1;
  write_le32(packet, 1066, static_cast<std::uint32_t>(time_us % 1000000));
  const auto seconds = static_cast<std::uint32_t>(time_us / 1000000);
  packet[1074] = static_cast<std::uint8_t>(seconds >> 24U);
  packet[1075] = static_cast<std::uint8_t>(seconds >> 16U);
  packet[1076] = static_cast<std::uint8_t>(seconds >> 8U);
  packet[1077] = static_cast<std::uint8_t>(seconds);
  write_le32(packet, 1078, sequence);
  write_le32(packet, 1082,
             joint_sensor_player::lidar_crc32(packet.data() + 1046, 36));
  return packet;
}

}  // namespace

TEST_CASE(player_merges_channels_on_absolute_deadlines) {
  auto defaults = joint_sensor_player::default_lidar_channels();
  joint_sensor_player::PlayerConfig config;
  config.start_delay_ms = 1;
  config.channels = {defaults[0], defaults[4]};
  config.channels[0].packet_interval_ns = 100;
  config.channels[1].packet_interval_ns = 250;
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[config.channels[0].topic] = {
      make_frame(1000, {1, 2}), make_frame(2000, {3})};
  catalog.frames[config.channels[1].topic] = {
      make_frame(1000, {4, 5}), make_frame(2000, {6})};

  FakeClock clock(100000);
  FakeSink sink;
  const auto result =
      joint_sensor_player::play_catalog(config, catalog, clock, sink);

  CHECK(result.ok);
  CHECK(result.packets_sent == 6);
  CHECK(sink.sent.size() == 6);
  CHECK(sink.sent[0].deadline == 1100000);
  CHECK(sink.sent[0].value == 1);
  CHECK(sink.sent[1].value == 4);
  CHECK(sink.sent[2].deadline == 1100100);
  CHECK(sink.sent[3].deadline == 1100250);
  CHECK(sink.sent[4].deadline == 1101000);
  CHECK(sink.sent[5].deadline == 1101000);
}

TEST_CASE(player_emits_existing_frame_stat_to_optional_observer) {
  auto defaults = joint_sensor_player::default_lidar_channels();
  joint_sensor_player::PlayerConfig config;
  config.start_delay_ms = 0;
  config.channels = {defaults[0]};
  config.channels[0].packet_interval_ns = 100;
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[config.channels[0].topic] = {make_frame(1000, {1, 2})};
  FakeClock clock(100);
  FakeSink sink;
  std::vector<joint_sensor_player::FrameStat> observed;

  const auto result = joint_sensor_player::play_catalog(
      config, catalog, clock, sink, nullptr, nullptr, nullptr, std::nullopt,
      [&](const auto& stat) { observed.push_back(stat); });

  CHECK(result.ok);
  CHECK(observed.size() == 1);
  CHECK(observed[0].slot == config.channels[0].slot);
  CHECK(observed[0].packet_count == 2);
  CHECK(observed[0].source_timestamp_ns == 1000);
}

TEST_CASE(player_stops_all_channels_on_send_failure) {
  auto defaults = joint_sensor_player::default_lidar_channels();
  joint_sensor_player::PlayerConfig config;
  config.start_delay_ms = 0;
  config.channels = {defaults[0], defaults[1]};
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[config.channels[0].topic] = {
      make_frame(1000, {1, 2, 3})};
  catalog.frames[config.channels[1].topic] = {
      make_frame(1000, {4, 5, 6})};
  FakeClock clock(100);
  FakeSink sink;
  sink.fail_after = 2;
  const auto result =
      joint_sensor_player::play_catalog(config, catalog, clock, sink);
  CHECK(!result.ok);
  CHECK(result.packets_sent == 2);
  CHECK(result.error == "injected failure");
}

TEST_CASE(player_stops_when_lateness_exceeds_configured_limit) {
  const auto defaults = joint_sensor_player::default_lidar_channels();
  joint_sensor_player::PlayerConfig config;
  config.start_delay_ms = 1;
  config.max_lateness_us = 20000;
  config.channels = {defaults[0]};
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[config.channels[0].topic] = {make_frame(1000, {1, 2})};
  FakeClock clock(100);
  clock.late_by_ns = 20001000;
  FakeSink sink;
  const auto result =
      joint_sensor_player::play_catalog(config, catalog, clock, sink);
  CHECK(!result.ok);
  CHECK(result.packets_sent == 0);
  CHECK(result.error.find("发送迟到") != std::string::npos);
}

TEST_CASE(player_runs_two_cycles_on_one_continuous_timeline) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  joint_sensor_player::PlayerConfig config;
  config.start_delay_ms = 1;
  config.loop_count = 2;
  config.sequence_mode = joint_sensor_player::SequenceMode::Preserve;
  config.channels = {channel};
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[channel.topic] = {
      make_frame(100000000, {1}), make_frame(200000000, {2})};
  FakeClock clock(100000);
  FakeSink sink;
  const auto result =
      joint_sensor_player::play_catalog(config, catalog, clock, sink);
  CHECK(result.ok);
  CHECK(result.loops_completed == 2);
  CHECK(result.packets_sent == 4);
  CHECK(sink.sent[0].deadline == 1100000);
  CHECK(sink.sent[1].deadline == 101100000);
  CHECK(sink.sent[2].deadline == 201100000);
  CHECK(sink.sent[3].deadline == 301100000);
}

TEST_CASE(infinite_loop_stops_normally_on_cancellation) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  joint_sensor_player::PlayerConfig config;
  config.start_delay_ms = 1;
  config.loop_count = 0;
  config.sequence_mode = joint_sensor_player::SequenceMode::Preserve;
  config.channels = {channel};
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[channel.topic] = {
      make_frame(100000000, {1}), make_frame(200000000, {2})};
  FakeClock clock(100000);
  FakeSink sink;
  joint_sensor_player::CancellationToken token;
  sink.cancel_token = &token;
  sink.cancel_after = 3;
  const auto result =
      joint_sensor_player::play_catalog(config, catalog, clock, sink, nullptr,
                                        &token);
  CHECK(result.ok);
  CHECK(result.cancelled);
  CHECK(result.loops_completed == 1);
  CHECK(result.packets_sent == 3);
  CHECK(result.stop_reason == "cancelled");
}

TEST_CASE(cancellation_token_observes_signal_safe_flag) {
  volatile std::sig_atomic_t signal_stop = 0;
  joint_sensor_player::CancellationToken token(&signal_stop);
  CHECK(!token.stop_requested());
  signal_stop = 1;
  CHECK(token.stop_requested());
}

TEST_CASE(player_rejects_initial_deadline_overflow) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  joint_sensor_player::PlayerConfig config;
  config.start_delay_ms = 1;
  config.channels = {channel};
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[channel.topic] = {make_frame(1000, {1})};
  FakeClock clock(std::numeric_limits<std::uint64_t>::max() - 10);
  FakeSink sink;
  const auto result =
      joint_sensor_player::play_catalog(config, catalog, clock, sink);
  CHECK(!result.ok);
  CHECK(result.packets_sent == 0);
  CHECK(result.error.find("起始期限溢出") != std::string::npos);
}

TEST_CASE(player_uses_explicit_shared_start_when_supplied) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  joint_sensor_player::PlayerConfig config;
  config.start_delay_ms = 999;
  config.channels = {channel};
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[channel.topic] = {make_frame(1000, {1})};
  FakeClock clock(500);
  FakeSink sink;

  const auto result = joint_sensor_player::play_catalog(
      config, catalog, clock, sink, nullptr, nullptr, nullptr, 10'000);

  CHECK(result.ok);
  CHECK(result.play_start_ns == 10'000);
  CHECK(sink.sent.size() == 1);
  CHECK(sink.sent.front().deadline == 10'000);
}

TEST_CASE(player_rejects_explicit_start_earlier_than_now) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  joint_sensor_player::PlayerConfig config;
  config.channels = {channel};
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[channel.topic] = {make_frame(1000, {1})};
  FakeClock clock(500);
  FakeSink sink;

  const auto result = joint_sensor_player::play_catalog(
      config, catalog, clock, sink, nullptr, nullptr, nullptr, 499);

  CHECK(!result.ok);
  CHECK(result.packets_sent == 0);
  CHECK(result.error.find("显式播放起点早于当前单调时间") !=
        std::string::npos);
}

TEST_CASE(player_long_wait_checks_cancellation_in_one_ms_absolute_steps) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  joint_sensor_player::PlayerConfig config;
  config.channels = {channel};
  joint_sensor_player::McapCatalog catalog;
  catalog.frames[channel.topic] = {make_frame(1000, {1})};
  FakeClock clock(0);
  FakeSink sink;
  joint_sensor_player::CancellationToken cancellation;
  clock.cancel_on_sleep = &cancellation;

  const auto result = joint_sensor_player::play_catalog(
      config, catalog, clock, sink, nullptr, &cancellation, nullptr,
      10'000'000);

  CHECK(result.ok);
  CHECK(result.cancelled);
  CHECK(sink.sent.empty());
  CHECK(clock.sleep_deadlines.size() == 1);
  CHECK(clock.sleep_deadlines.front() <= 1'000'000);
}

TEST_CASE(player_uses_emulator_rewriter_without_mutating_catalog) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  joint_sensor_player::PlayerConfig config;
  config.start_delay_ms = 0;
  config.loop_count = 2;
  config.channels = {channel};
  joint_sensor_player::McapCatalog catalog;
  joint_sensor_player::LidarFrame first;
  first.source_timestamp_ns = 100000000ULL;
  first.packets = {main_packet(1000000ULL, 10)};
  joint_sensor_player::LidarFrame second;
  second.source_timestamp_ns = 200000000ULL;
  second.packets = {main_packet(1050000ULL, 11)};
  catalog.frames[channel.topic] = {first, second};
  const auto original = catalog.frames[channel.topic][0].packets[0];
  const joint_sensor_player::PlaybackWindow window{100000000ULL,
                                                   200000000ULL};
  const auto rewriter = joint_sensor_player::LidarPacketRewriter::build(
      catalog, {channel}, window, 200000000ULL, 1792549044000000ULL);
  FakeClock clock(1000);
  FakeSink sink;
  const auto result = joint_sensor_player::play_catalog(
      config, catalog, clock, sink, nullptr, nullptr, &rewriter);
  CHECK(result.ok);
  CHECK(sink.sent.size() == 4);
  const auto cycle0 =
      joint_sensor_player::inspect_lidar_packet(sink.sent[0].payload);
  const auto cycle1 =
      joint_sensor_player::inspect_lidar_packet(sink.sent[2].payload);
  CHECK(cycle0.sensor_time_us == 1792549044000000ULL);
  CHECK(cycle1.sensor_time_us == 1792549044200000ULL);
  CHECK(cycle0.sequence == 10);
  CHECK(cycle1.sequence == 12);
  CHECK(catalog.frames[channel.topic][0].packets[0] == original);
}
