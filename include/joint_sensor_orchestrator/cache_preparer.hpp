#pragma once

#include "joint_sensor_orchestrator/orchestrator_config.hpp"
#include "joint_sensor_orchestrator/video_cache.hpp"

#include <cstdint>
#include <string>

namespace joint_sensor_orchestrator {

VideoCache validate_cache_builder_output(
    const OrchestratorConfig& config, const std::string& stdout_text,
    std::uint64_t expected_frame_bytes = kVideoFrameBytes);

}  // namespace joint_sensor_orchestrator
