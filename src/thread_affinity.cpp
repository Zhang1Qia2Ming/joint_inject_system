// 读 /sys 里的在线 CPU 拓扑，给视频写卡、生产和预读各留一个物理核。
// 雷达不绑。核还够四路时，环视占用编号最高的四个逻辑 CPU。

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "joint_sensor_orchestrator/thread_affinity.hpp"

#include <sched.h>
#include <unistd.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <string>
#include <utility>

namespace joint_sensor_orchestrator {

SendAffinity plan_send_affinity(const std::vector<std::vector<int>>& cores) {
  SendAffinity plan;
  std::vector<int> chosen;
  chosen.reserve(cores.size());
  for (std::size_t index = 1; index < cores.size(); ++index) {
    if (cores[index].empty()) continue;
    chosen.push_back(cores[index].front());
  }
  // 3 writers + 3 producers + filler. Radar stays unpinned: pinning its
  // sender onto one CPU made the first second ~85 ms late, because that
  // CPU also serves the NIC queues.
  if (chosen.size() < 7) return plan;
  plan.enabled = true;
  for (std::size_t index = 0; index < 3; ++index) {
    plan.video_consumers[index] = chosen[index];
    plan.video_producers[index] = chosen[index + 3];
  }
  plan.filler = chosen[6];
  if (chosen.size() >= 11) {
    for (std::size_t index = 0; index < plan.shadow.size(); ++index) {
      plan.shadow[index] = chosen[chosen.size() - plan.shadow.size() + index];
    }
  }
  return plan;
}

std::vector<std::vector<int>> online_physical_cores() {
  const long cpu_count = ::sysconf(_SC_NPROCESSORS_CONF);
  if (cpu_count <= 0) return {};
  std::map<std::pair<int, int>, std::vector<int>> grouped;
  for (long cpu = 0; cpu < cpu_count; ++cpu) {
    const auto dir =
        "/sys/devices/system/cpu/cpu" + std::to_string(cpu);
    std::ifstream online(dir + "/online");
    if (online) {
      int flag = 0;
      online >> flag;
      if (!online || flag == 0) continue;
    } else {
      std::ifstream present(dir + "/topology/core_id");
      if (!present) continue;
    }
    std::ifstream core_file(dir + "/topology/core_id");
    std::ifstream package_file(dir + "/topology/physical_package_id");
    int core_id = -1;
    int package_id = -1;
    if (!core_file || !package_file) continue;
    core_file >> core_id;
    package_file >> package_id;
    if (core_id < 0 || package_id < 0) continue;
    grouped[{package_id, core_id}].push_back(static_cast<int>(cpu));
  }
  std::vector<std::vector<int>> cores;
  cores.reserve(grouped.size());
  for (auto& [ignored, cpus] : grouped) {
    (void)ignored;
    std::sort(cpus.begin(), cpus.end());
    cores.push_back(std::move(cpus));
  }
  return cores;
}

bool pin_current_thread(int cpu) {
  if (cpu < 0) return false;
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  return ::sched_setaffinity(0, sizeof(set), &set) == 0;
}

}  // namespace joint_sensor_orchestrator
