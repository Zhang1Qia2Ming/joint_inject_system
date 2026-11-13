#include "joint_sensor_orchestrator/dataset_validation.hpp"

#include "joint_sensor_player/timeline.hpp"

#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>

namespace joint_sensor_orchestrator {
namespace {

template <typename RadarCatalog>
JointDatasetValidation validate_joint_dataset_impl(
    const OrchestratorConfig& config, const VideoCache& cache,
    const RadarCatalog& radar_catalog) {
  JointDatasetValidation result;
  std::set<std::string> radar_topics;
  for (const auto& channel : config.lidar.channels) {
    if (!channel.enabled || !radar_topics.insert(channel.topic).second ||
        !radar_catalog.frames.contains(channel.topic) ||
        radar_catalog.frames.at(channel.topic).empty()) {
      throw std::runtime_error(
          "共同窗口必须包含8个唯一且非空的雷达topic");
    }
    result.radar_topics.push_back(channel.topic);
  }
  if (result.radar_topics.size() != 8) {
    throw std::runtime_error("共同窗口雷达参与者必须正好为8");
  }

  result.window = {0, std::numeric_limits<std::uint64_t>::max()};
  for (const auto& topic : result.radar_topics) {
    const auto& frames = radar_catalog.frames.at(topic);
    const auto [first, last] = std::minmax_element(
        frames.begin(), frames.end(),
        [](const auto& left, const auto& right) {
          return left.bag_log_time_ns < right.bag_log_time_ns;
        });
    result.window.start_ns =
        std::max(result.window.start_ns, first->bag_log_time_ns);
    result.window.end_ns =
        std::min(result.window.end_ns, last->bag_log_time_ns);
  }
  std::set<std::string> video_topics;
  for (std::size_t index = 0; index < config.lanes.size(); ++index) {
    const auto& configured = config.lanes[index];
    const auto& lane = cache.lanes[index];
    if (lane.topic != configured.topic ||
        !video_topics.insert(lane.topic).second ||
        lane.timestamps_ns.empty()) {
      throw std::runtime_error(
          "共同窗口必须包含3个唯一且非空的视频topic");
    }
    result.video_topics.push_back(lane.topic);
    result.window.start_ns =
        std::max(result.window.start_ns, lane.timestamps_ns.front());
    result.window.end_ns =
        std::min(result.window.end_ns, lane.timestamps_ns.back());
  }
  if (result.video_topics.size() != 3 ||
      result.window.start_ns > result.window.end_ns) {
    throw std::runtime_error("11路参与者不存在有效共同源时间窗口");
  }
  return result;
}

}  // namespace

JointDatasetValidation validate_joint_dataset(
    const OrchestratorConfig& config, const VideoCache& cache,
    joint_sensor_player::McapCatalog& radar_catalog) {
  return validate_joint_dataset_impl(config, cache, radar_catalog);
}

JointDatasetValidation validate_joint_dataset(
    const OrchestratorConfig& config, const VideoCache& cache,
    const joint_sensor_player::LightweightMcapCatalog& radar_catalog) {
  return validate_joint_dataset_impl(config, cache, radar_catalog);
}

}  // namespace joint_sensor_orchestrator
