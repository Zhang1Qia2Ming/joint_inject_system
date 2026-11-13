#include "joint_sensor_orchestrator/dataset_validation.hpp"
#include "test_framework.hpp"

namespace {

joint_sensor_orchestrator::OrchestratorConfig participant_config() {
  joint_sensor_orchestrator::OrchestratorConfig config;
  const auto defaults = joint_sensor_player::default_lidar_channels();
  config.lidar.channels.assign(defaults.begin(), defaults.end());
  return config;
}

joint_sensor_player::McapCatalog participant_catalog(
    const joint_sensor_orchestrator::OrchestratorConfig& config) {
  joint_sensor_player::McapCatalog catalog;
  for (const auto& channel : config.lidar.channels) {
    catalog.frames[channel.topic] = {
        {100, 100, 1, {}, {}},
        {200, 200, 2, {}, {}},
    };
  }
  return catalog;
}

joint_sensor_orchestrator::VideoCache participant_cache(
    const joint_sensor_orchestrator::OrchestratorConfig& config) {
  joint_sensor_orchestrator::VideoCache cache;
  for (std::size_t index = 0; index < config.lanes.size(); ++index) {
    cache.lanes[index].name = config.lanes[index].name;
    cache.lanes[index].topic = config.lanes[index].topic;
    cache.lanes[index].timestamps_ns = {110, 190};
  }
  return cache;
}

}  // namespace

TEST_CASE(common_window_reports_exactly_eight_radar_and_three_video_topics) {
  const auto config = participant_config();
  auto catalog = participant_catalog(config);
  const auto cache = participant_cache(config);
  const auto result = joint_sensor_orchestrator::validate_joint_dataset(
      config, cache, catalog);
  CHECK(result.window.start_ns == 110);
  CHECK(result.window.end_ns == 190);
  CHECK(result.radar_topics.size() == 8);
  CHECK(result.video_topics.size() == 3);
  CHECK(result.radar_topics.size() + result.video_topics.size() == 11);
}

TEST_CASE(common_window_uses_radar_bag_log_time_when_payload_clock_differs) {
  const auto config = participant_config();
  auto catalog = participant_catalog(config);
  constexpr std::uint64_t bag_start = 1'792'549'024'000'000'000ULL;
  constexpr std::uint64_t bag_end = 1'792'549'064'000'000'000ULL;
  for (auto& [topic, frames] : catalog.frames) {
    (void)topic;
    frames = {
        {2'076'000'000'000ULL, bag_start, 1, {}, {}},
        {2'116'000'000'000ULL, bag_end, 2, {}, {}},
    };
  }
  auto cache = participant_cache(config);
  for (auto& lane : cache.lanes) {
    lane.timestamps_ns = {bag_start + 1'000'000'000ULL,
                          bag_end - 1'000'000'000ULL};
  }

  const auto result = joint_sensor_orchestrator::validate_joint_dataset(
      config, cache, catalog);

  CHECK(result.window.start_ns == bag_start + 1'000'000'000ULL);
  CHECK(result.window.end_ns == bag_end - 1'000'000'000ULL);
}

TEST_CASE(common_window_rejects_any_missing_radar_or_video_participant) {
  const auto config = participant_config();
  auto catalog = participant_catalog(config);
  auto cache = participant_cache(config);
  catalog.frames.erase(config.lidar.channels.front().topic);
  CHECK_THROWS(joint_sensor_orchestrator::validate_joint_dataset(
      config, cache, catalog));

  catalog = participant_catalog(config);
  cache.lanes[2].timestamps_ns.clear();
  CHECK_THROWS(joint_sensor_orchestrator::validate_joint_dataset(
      config, cache, catalog));
}

TEST_CASE(common_window_accepts_payload_free_lightweight_radar_catalog) {
  const auto config = participant_config();
  joint_sensor_player::LightweightMcapCatalog catalog;
  for (const auto& channel : config.lidar.channels) {
    catalog.frames[channel.topic] = {
        {.source_timestamp_ns = 100, .bag_log_time_ns = 100,
         .packet_count = 1},
        {.source_timestamp_ns = 200, .bag_log_time_ns = 200,
         .packet_count = 1},
    };
  }
  const auto result = joint_sensor_orchestrator::validate_joint_dataset(
      config, participant_cache(config), catalog);
  CHECK(result.window.start_ns == 110);
  CHECK(result.window.end_ns == 190);
  CHECK(catalog.retained_payload_bytes == 0);
}
