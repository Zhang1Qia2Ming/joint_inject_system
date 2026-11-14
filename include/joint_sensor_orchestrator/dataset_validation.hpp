#pragma once

// 开播前核对视频缓存和雷达目录盖住同一段源时间。
// 交出来的 SourceWindow 就是后面所有槽和包期限的共同窗口。

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
