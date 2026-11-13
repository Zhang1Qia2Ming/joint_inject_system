#include "joint_sensor_player/timeline.hpp"
#include "test_framework.hpp"

using joint_sensor_player::LidarFrame;
using joint_sensor_player::McapCatalog;
using joint_sensor_player::build_playback_window;
using joint_sensor_player::default_lidar_channels;
using joint_sensor_player::nominal_frame_period_ns;
using joint_sensor_player::packet_deadline_ns;

namespace {

LidarFrame frame(std::uint64_t timestamp) {
  LidarFrame value;
  value.source_timestamp_ns = timestamp;
  value.packets = {{1}};
  return value;
}

}  // namespace

TEST_CASE(strict_window_uses_common_intersection) {
  const auto defaults = default_lidar_channels();
  std::vector<joint_sensor_player::LidarChannelConfig> channels{
      defaults[0], defaults[1]};
  McapCatalog catalog;
  catalog.frames[channels[0].topic] = {frame(100), frame(300)};
  catalog.frames[channels[1].topic] = {frame(120), frame(280)};
  const auto window = build_playback_window(catalog, channels, false);
  CHECK(window.source_start_ns == 120);
  CHECK(window.source_end_ns == 280);
  CHECK(window.enabled_topics == 2);
}

TEST_CASE(strict_window_rejects_missing_channel) {
  const auto defaults = default_lidar_channels();
  std::vector<joint_sensor_player::LidarChannelConfig> channels{
      defaults[0], defaults[1]};
  McapCatalog catalog;
  catalog.frames[channels[0].topic] = {frame(100), frame(300)};
  CHECK_THROWS(build_playback_window(catalog, channels, false));
}

TEST_CASE(partial_window_accepts_present_channel) {
  const auto defaults = default_lidar_channels();
  std::vector<joint_sensor_player::LidarChannelConfig> channels{
      defaults[0], defaults[1]};
  McapCatalog catalog;
  catalog.frames[channels[0].topic] = {frame(100), frame(300)};
  const auto window = build_playback_window(catalog, channels, true);
  CHECK(window.enabled_topics == 1);
}

TEST_CASE(packet_deadline_preserves_source_time_and_spacing) {
  const std::uint64_t play_start = 1000000;
  const std::uint64_t source_start = 500000;
  CHECK(packet_deadline_ns(play_start, source_start, 600000, 3, 83200, 0) ==
        1349600);
  CHECK(packet_deadline_ns(play_start, source_start, 600000, 2, 193300,
                           -100) == 1386600);
}

TEST_CASE(nominal_period_uses_median_ten_hz_interval) {
  const auto defaults = default_lidar_channels();
  std::vector<joint_sensor_player::LidarChannelConfig> channels{
      defaults[0], defaults[1]};
  McapCatalog catalog;
  catalog.frames[channels[0].topic] = {
      frame(100000000), frame(200000000), frame(300000000)};
  catalog.frames[channels[1].topic] = {
      frame(110000000), frame(210000100), frame(310000100)};
  CHECK(nominal_frame_period_ns(catalog, channels) == 100000000);
}

TEST_CASE(nominal_period_rejects_non_ten_hz_material) {
  const auto channel = default_lidar_channels()[0];
  McapCatalog catalog;
  catalog.frames[channel.topic] = {
      frame(100000000), frame(150000000), frame(200000000)};
  CHECK_THROWS(nominal_frame_period_ns(catalog, {channel}));
}
