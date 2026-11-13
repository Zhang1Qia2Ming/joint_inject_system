#include "joint_sensor_player/packet_time_rewriter.hpp"
#include "joint_sensor_player/config.hpp"
#include "joint_sensor_player/mcap.hpp"
#include "joint_sensor_player/timeline.hpp"
#include "test_framework.hpp"

#include <cstring>
#include <limits>
#include <vector>

namespace {

void write_le32(std::vector<std::uint8_t>& packet, std::size_t offset,
                std::uint32_t value) {
  std::memcpy(packet.data() + offset, &value, sizeof(value));
}

std::uint32_t read_le32(const std::vector<std::uint8_t>& packet,
                        std::size_t offset) {
  std::uint32_t value{};
  std::memcpy(&value, packet.data() + offset, sizeof(value));
  return value;
}

void write_seconds(std::vector<std::uint8_t>& packet, std::size_t offset,
                   std::uint32_t seconds) {
  packet[offset] = 0;
  packet[offset + 1] = 0;
  packet[offset + 2] = static_cast<std::uint8_t>(seconds >> 24U);
  packet[offset + 3] = static_cast<std::uint8_t>(seconds >> 16U);
  packet[offset + 4] = static_cast<std::uint8_t>(seconds >> 8U);
  packet[offset + 5] = static_cast<std::uint8_t>(seconds);
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
  write_seconds(packet, 1072,
                static_cast<std::uint32_t>(time_us / 1000000));
  write_le32(packet, 1078, sequence);
  write_le32(packet, 1082,
             joint_sensor_player::lidar_crc32(packet.data() + 1046, 36));
  return packet;
}

std::vector<std::uint8_t> short_packet(std::uint64_t time_us,
                                       std::uint32_t sequence) {
  std::vector<std::uint8_t> packet(554);
  packet[0] = 0xEE;
  packet[1] = 0xFF;
  packet[2] = 0x07;
  packet[3] = 0x03;
  write_seconds(packet, 519,
                static_cast<std::uint32_t>(time_us / 1000000));
  write_le32(packet, 525, static_cast<std::uint32_t>(time_us % 1000000));
  packet[529] = 0x42;
  write_le32(packet, 530, sequence);
  return packet;
}

}  // namespace

TEST_CASE(packet_time_parser_recognizes_real_protocol_layouts) {
  const auto main = joint_sensor_player::inspect_lidar_packet(
      main_packet(2076800592ULL, 12430788));
  CHECK(main.protocol ==
        joint_sensor_player::LidarPacketProtocol::MainUdp43);
  CHECK(main.sensor_time_us == 2076800592ULL);
  CHECK(main.sequence == 12430788);

  const auto short_info = joint_sensor_player::inspect_lidar_packet(
      short_packet(2076753396ULL, 10209281));
  CHECK(short_info.protocol ==
        joint_sensor_player::LidarPacketProtocol::ShortUdp73);
  CHECK(short_info.sensor_time_us == 2076753396ULL);
  CHECK(short_info.sequence == 10209281);
}

TEST_CASE(packet_rewrite_updates_time_sequence_and_main_crc) {
  auto packet = main_packet(2076800592ULL, 10);
  const auto old_crc = read_le32(packet, 1082);
  joint_sensor_player::rewrite_lidar_packet(packet, 1792549044123456ULL, 77);
  const auto info = joint_sensor_player::inspect_lidar_packet(packet);
  CHECK(info.sensor_time_us == 1792549044123456ULL);
  CHECK(info.sequence == 77);
  CHECK(read_le32(packet, 1082) != old_crc);
  CHECK(read_le32(packet, 1082) ==
        joint_sensor_player::lidar_crc32(packet.data() + 1046, 36));
}

TEST_CASE(short_rewrite_preserves_non_time_tail_fields) {
  auto packet = short_packet(2076753396ULL, 12);
  const auto suffix = std::vector<std::uint8_t>(packet.begin() + 534,
                                                packet.end());
  joint_sensor_player::rewrite_lidar_packet(packet, 1792549044999999ULL, 99);
  const auto info = joint_sensor_player::inspect_lidar_packet(packet);
  CHECK(info.sensor_time_us == 1792549044999999ULL);
  CHECK(info.sequence == 99);
  CHECK(packet[529] == 0x42);
  CHECK(std::vector<std::uint8_t>(packet.begin() + 534, packet.end()) ==
        suffix);
}

TEST_CASE(packet_rewrite_handles_second_day_boundaries_and_rejects_overflow) {
  auto packet = main_packet(2076800592ULL, 1);
  const std::uint64_t before_midnight = 1792627199999999ULL;
  joint_sensor_player::rewrite_lidar_packet(packet, before_midnight, 2);
  CHECK(joint_sensor_player::inspect_lidar_packet(packet).sensor_time_us ==
        before_midnight);
  joint_sensor_player::rewrite_lidar_packet(packet, before_midnight + 2, 3);
  CHECK(joint_sensor_player::inspect_lidar_packet(packet).sensor_time_us ==
        before_midnight + 2);
  CHECK_THROWS(joint_sensor_player::rewrite_lidar_packet(
      packet,
      (static_cast<std::uint64_t>(
           std::numeric_limits<std::uint32_t>::max()) +
       1) *
          1000000ULL,
      4));
}

TEST_CASE(packet_parser_rejects_unknown_and_corrupt_layouts) {
  CHECK_THROWS(joint_sensor_player::inspect_lidar_packet(
      std::vector<std::uint8_t>(554)));
  auto bad_main = main_packet(2076800592ULL, 1);
  bad_main[1082] ^= 0xFF;
  CHECK_THROWS(joint_sensor_player::inspect_lidar_packet(bad_main));
}

TEST_CASE(emulation_plan_preserves_relative_time_and_advances_cycles) {
  const auto defaults = joint_sensor_player::default_lidar_channels();
  const std::vector<joint_sensor_player::LidarChannelConfig> channels{
      defaults[0], defaults[4]};
  joint_sensor_player::McapCatalog catalog;
  joint_sensor_player::LidarFrame main_frame;
  main_frame.source_timestamp_ns = 1000000000ULL;
  main_frame.packets = {main_packet(1000100ULL, 10),
                        main_packet(1052000ULL, 11)};
  catalog.frames[channels[0].topic] = {main_frame};
  joint_sensor_player::LidarFrame short_frame;
  short_frame.source_timestamp_ns = 1000000000ULL;
  short_frame.packets = {short_packet(950000ULL, 20),
                         short_packet(1049000ULL, 21)};
  catalog.frames[channels[1].topic] = {short_frame};

  const joint_sensor_player::PlaybackWindow window{1000000000ULL,
                                                   1000000000ULL};
  const auto rewriter = joint_sensor_player::LidarPacketRewriter::build(
      catalog, channels, window, 200000000ULL, 1792549044000000ULL);
  CHECK(rewriter.source_origin_us() == 950000ULL);
  CHECK(rewriter.cycle_period_us() == 200000ULL);
  CHECK(rewriter.channel_ranges().at(channels[0].topic).first_us ==
        1000100ULL);
  CHECK(rewriter.channel_ranges().at(channels[0].topic).last_us ==
        1052000ULL);

  const auto cycle0 = rewriter.rewrite(
      channels[0].topic, main_frame.packets[0], 0);
  const auto cycle1 = rewriter.rewrite(
      channels[0].topic, main_frame.packets[0], 1);
  const auto first = joint_sensor_player::inspect_lidar_packet(cycle0);
  const auto second = joint_sensor_player::inspect_lidar_packet(cycle1);
  CHECK(first.sensor_time_us == 1792549044050100ULL);
  CHECK(second.sensor_time_us == first.sensor_time_us + 200000ULL);
  CHECK(first.sequence == 10);
  CHECK(second.sequence == 12);
  CHECK(main_frame.packets[0] ==
        catalog.frames[channels[0].topic][0].packets[0]);
}

TEST_CASE(single_pass_plan_rewrites_without_cross_cycle_span_constraint) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  joint_sensor_player::McapCatalog catalog;
  joint_sensor_player::LidarFrame frame;
  frame.source_timestamp_ns = 100;
  frame.packets = {main_packet(1'000'000ULL, 1),
                   main_packet(1'300'000ULL, 2)};
  catalog.frames[channel.topic] = {frame};
  const joint_sensor_player::PlaybackWindow window{100, 100};

  const auto rewriter =
      joint_sensor_player::LidarPacketRewriter::build_single_pass(
          catalog, {channel}, window, 1'792'549'044'000'000ULL);
  const auto rewritten =
      rewriter.rewrite(channel.topic, frame.packets.back(), 0);

  CHECK(joint_sensor_player::inspect_lidar_packet(rewritten).sensor_time_us ==
        1'792'549'044'300'000ULL);
  CHECK_THROWS(rewriter.rewrite(channel.topic, frame.packets.front(), 1));
}

TEST_CASE(single_pass_plan_still_rejects_backward_packet_time) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  joint_sensor_player::McapCatalog catalog;
  joint_sensor_player::LidarFrame frame;
  frame.source_timestamp_ns = 100;
  frame.packets = {main_packet(2'000'000ULL, 1),
                   main_packet(1'900'000ULL, 2)};
  catalog.frames[channel.topic] = {frame};
  const joint_sensor_player::PlaybackWindow window{100, 100};

  CHECK_THROWS(joint_sensor_player::LidarPacketRewriter::build_single_pass(
      catalog, {channel}, window, 1'792'549'044'000'000ULL));
}

TEST_CASE(looping_plan_still_rejects_payload_span_reaching_cycle_period) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  joint_sensor_player::McapCatalog catalog;
  joint_sensor_player::LidarFrame frame;
  frame.source_timestamp_ns = 100;
  frame.packets = {main_packet(1'000'000ULL, 1),
                   main_packet(1'200'000ULL, 2)};
  catalog.frames[channel.topic] = {frame};
  const joint_sensor_player::PlaybackWindow window{100, 100};

  CHECK_THROWS(joint_sensor_player::LidarPacketRewriter::build(
      catalog, {channel}, window, 200'000'000ULL,
      1'792'549'044'000'000ULL));
}

TEST_CASE(emulation_plan_rejects_backward_time_and_wrong_channel_protocol) {
  const auto defaults = joint_sensor_player::default_lidar_channels();
  const auto main_channel = defaults[0];
  joint_sensor_player::McapCatalog backward;
  joint_sensor_player::LidarFrame frame;
  frame.source_timestamp_ns = 100;
  frame.packets = {main_packet(2000000ULL, 1),
                   main_packet(1900000ULL, 2)};
  backward.frames[main_channel.topic] = {frame};
  const joint_sensor_player::PlaybackWindow window{100, 100};
  CHECK_THROWS(joint_sensor_player::LidarPacketRewriter::build(
      backward, {main_channel}, window, 100000000ULL,
      1792549044000000ULL));

  joint_sensor_player::McapCatalog wrong;
  frame.packets = {short_packet(2000000ULL, 1)};
  wrong.frames[main_channel.topic] = {frame};
  CHECK_THROWS(joint_sensor_player::LidarPacketRewriter::build(
      wrong, {main_channel}, window, 100000000ULL,
      1792549044000000ULL));
}
