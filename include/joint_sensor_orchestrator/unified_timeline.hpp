#pragma once

#include <cstdint>
#include <vector>

namespace joint_sensor_orchestrator {

struct SourceWindow {
  std::uint64_t start_ns{};
  std::uint64_t end_ns{};
};

struct VideoSlot {
  std::uint64_t source_index{};
  std::uint64_t output_index{};
  std::uint64_t grid_index{};
  std::uint64_t source_time_ns{};
  std::uint64_t slot_time_ns{};
  std::uint64_t relative_ns{};
  std::uint64_t deadline_ns{};
};

std::vector<VideoSlot> build_video_slots(
    const std::vector<std::uint64_t>& source_timestamps_ns,
    unsigned source_fps, unsigned output_fps, SourceWindow common_window,
    std::uint64_t play_start_ns);

}  // namespace joint_sensor_orchestrator
