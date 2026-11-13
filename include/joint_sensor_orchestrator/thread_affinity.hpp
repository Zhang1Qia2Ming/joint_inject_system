#pragma once

#include <array>
#include <vector>

namespace joint_sensor_orchestrator {

struct SendAffinity {
  bool enabled{false};
  std::array<int, 3> video_consumers{-1, -1, -1};
  std::array<int, 3> video_producers{-1, -1, -1};
  int filler{-1};
  int radar_sender{-1};
  int radar_reader{-1};
};

// cores[i] lists the logical CPUs of one physical core. The first core is
// left alone. Video writers, producers, and the disk filler each take one
// logical CPU from a different physical core. Radar is left unpinned.
SendAffinity plan_send_affinity(const std::vector<std::vector<int>>& cores);

std::vector<std::vector<int>> online_physical_cores();

bool pin_current_thread(int cpu);

}  // namespace joint_sensor_orchestrator
