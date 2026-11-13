#include "joint_sensor_player/packet_sequence.hpp"
#include "test_framework.hpp"

#include <cstring>

namespace {

void write_u32(std::vector<std::uint8_t>& packet, std::size_t offset,
               std::uint32_t value) {
  std::memcpy(packet.data() + offset, &value, sizeof(value));
}

std::uint32_t read_u32(const std::vector<std::uint8_t>& packet,
                       std::size_t offset) {
  std::uint32_t value{};
  std::memcpy(&value, packet.data() + offset, sizeof(value));
  return value;
}

std::vector<std::uint8_t> main_packet(std::uint32_t sequence) {
  std::vector<std::uint8_t> packet(1118);
  packet[0] = 0xEE;
  packet[1] = 0xFF;
  packet[2] = 0x04;
  packet[3] = 0x03;
  packet[6] = 128;
  packet[7] = 2;
  packet[11] = 1;
  write_u32(packet, 1078, sequence);
  return packet;
}

std::vector<std::uint8_t> short_packet(std::uint32_t sequence) {
  std::vector<std::uint8_t> packet(554);
  packet[0] = 0xEE;
  packet[1] = 0xFF;
  packet[2] = 0x07;
  packet[3] = 0x03;
  write_u32(packet, 530, sequence);
  return packet;
}

joint_sensor_player::McapCatalog catalog_for(
    const std::string& topic,
    std::initializer_list<std::vector<std::uint8_t>> packets) {
  joint_sensor_player::McapCatalog catalog;
  joint_sensor_player::LidarFrame frame;
  frame.source_timestamp_ns = 1;
  frame.packets = packets;
  catalog.frames[topic] = {std::move(frame)};
  return catalog;
}

}  // namespace

TEST_CASE(sequence_plan_recognizes_main_and_preserves_internal_gaps) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  auto catalog =
      catalog_for(channel.topic,
                  {main_packet(10), main_packet(11), main_packet(20)});
  const auto plans = joint_sensor_player::build_sequence_plans(
      catalog, {channel}, joint_sensor_player::SequenceMode::Continuous);
  const auto& plan = plans.at(channel.topic);
  CHECK(plan.sequence_offset == 1078);
  CHECK(plan.cycle_span == 11);
  for (auto& packet : catalog.frames[channel.topic][0].packets) {
    joint_sensor_player::advance_packet_sequence(packet, plan);
  }
  CHECK(read_u32(catalog.frames[channel.topic][0].packets[0], 1078) == 21);
  CHECK(read_u32(catalog.frames[channel.topic][0].packets[1], 1078) == 22);
  CHECK(read_u32(catalog.frames[channel.topic][0].packets[2], 1078) == 31);
}

TEST_CASE(sequence_plan_recognizes_short_and_wraps_uint32) {
  const auto channel = joint_sensor_player::default_lidar_channels()[4];
  auto catalog = catalog_for(
      channel.topic,
      {short_packet(0xFFFFFFFEU), short_packet(0xFFFFFFFFU), short_packet(0)});
  const auto plans = joint_sensor_player::build_sequence_plans(
      catalog, {channel}, joint_sensor_player::SequenceMode::Continuous);
  const auto& plan = plans.at(channel.topic);
  CHECK(plan.sequence_offset == 530);
  CHECK(plan.cycle_span == 3);
  auto& first = catalog.frames[channel.topic][0].packets[0];
  joint_sensor_player::advance_packet_sequence(first, plan);
  CHECK(read_u32(first, 530) == 1);
}

TEST_CASE(sequence_plan_rejects_unknown_or_backward_packets) {
  const auto channel = joint_sensor_player::default_lidar_channels()[4];
  auto unknown = catalog_for(channel.topic, {std::vector<std::uint8_t>(554)});
  CHECK_THROWS(joint_sensor_player::build_sequence_plans(
      unknown, {channel}, joint_sensor_player::SequenceMode::Continuous));
  auto backward =
      catalog_for(channel.topic, {short_packet(100), short_packet(90)});
  CHECK_THROWS(joint_sensor_player::build_sequence_plans(
      backward, {channel}, joint_sensor_player::SequenceMode::Continuous));
}

TEST_CASE(preserve_mode_does_not_require_or_modify_sequence) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  auto catalog =
      catalog_for(channel.topic, {std::vector<std::uint8_t>{1, 2, 3}});
  const auto original = catalog.frames[channel.topic][0].packets[0];
  const auto plans = joint_sensor_player::build_sequence_plans(
      catalog, {channel}, joint_sensor_player::SequenceMode::Preserve);
  CHECK(plans.empty());
  CHECK(catalog.frames[channel.topic][0].packets[0] == original);
}

TEST_CASE(sequence_span_uses_only_common_playback_window) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  joint_sensor_player::McapCatalog catalog;
  for (const auto& [timestamp, sequence] :
       {std::pair{50ULL, 1U}, std::pair{100ULL, 10U},
        std::pair{200ULL, 20U}, std::pair{300ULL, 30U}}) {
    joint_sensor_player::LidarFrame frame;
    frame.source_timestamp_ns = timestamp;
    frame.packets = {main_packet(sequence)};
    catalog.frames[channel.topic].push_back(std::move(frame));
  }
  const auto plans = joint_sensor_player::build_sequence_plans(
      catalog, {channel}, joint_sensor_player::SequenceMode::Continuous, 100,
      200);
  CHECK(plans.at(channel.topic).cycle_span == 11);
}

TEST_CASE(sequence_validation_accepts_large_forward_gap) {
  const auto channel = joint_sensor_player::default_lidar_channels()[0];
  auto catalog =
      catalog_for(channel.topic,
                  {main_packet(10), main_packet(2000010)});
  const auto plans = joint_sensor_player::build_sequence_plans(
      catalog, {channel}, joint_sensor_player::SequenceMode::Continuous);
  CHECK(plans.at(channel.topic).cycle_span == 2000001);
}
