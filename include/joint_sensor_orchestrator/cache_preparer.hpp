#pragma once

// 调用外部缓存构建程序之后，核对其标准输出和磁盘上的清单。
// 对不上就拒绝把这份缓存交给播放器。

#include "joint_sensor_orchestrator/orchestrator_config.hpp"
#include "joint_sensor_orchestrator/video_cache.hpp"

#include <cstdint>
#include <string>

namespace joint_sensor_orchestrator {

VideoCache validate_cache_builder_output(
    const OrchestratorConfig& config, const std::string& stdout_text,
    std::uint64_t expected_frame_bytes = kVideoFrameBytes);

}  // namespace joint_sensor_orchestrator
