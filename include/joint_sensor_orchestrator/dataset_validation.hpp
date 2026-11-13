#pragma once

#include "joint_sensor_orchestrator/orchestrator_config.hpp"
#include "joint_sensor_orchestrator/unified_timeline.hpp"
#include "joint_sensor_orchestrator/video_cache.hpp"
#include "joint_sensor_player/mcap.hpp"

#include <string>
#include <vector>

namespace joint_sensor_orchestrator {

struct JointDatasetValidation {
  SourceWindow window;
  std::vector<std::string> radar_topics;
  std::vector<std::string> video_topics;
};

JointDatasetValidation validate_joint_dataset(
    const OrchestratorConfig& config, const VideoCache& cache,
    joint_sensor_player::McapCatalog& radar_catalog);

JointDatasetValidation validate_joint_dataset(
    const OrchestratorConfig& config, const VideoCache& cache,
    const joint_sensor_player::LightweightMcapCatalog& radar_catalog);

}  // namespace joint_sensor_orchestrator
