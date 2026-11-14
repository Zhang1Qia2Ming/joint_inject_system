#include "joint_sensor_orchestrator/thread_affinity.hpp"
#include "test_framework.hpp"

#include <set>
#include <vector>

namespace {

bool shares_a_core(const std::vector<std::vector<int>>& cores, int left,
                   int right) {
  if (left < 0 || right < 0 || left == right) return left >= 0 && left == right;
  for (const auto& core : cores) {
    bool has_left = false;
    bool has_right = false;
    for (const int cpu : core) {
      if (cpu == left) has_left = true;
      if (cpu == right) has_right = true;
    }
    if (has_left && has_right) return true;
  }
  return false;
}

}  // namespace

TEST_CASE(send_affinity_puts_each_role_on_its_own_physical_core) {
  std::vector<std::vector<int>> cores;
  for (int index = 0; index < 12; ++index) {
    cores.push_back({index * 2, index * 2 + 1});
  }
  const auto plan = joint_sensor_orchestrator::plan_send_affinity(cores);
  CHECK(plan.enabled);
  CHECK(plan.radar_sender < 0);
  CHECK(plan.radar_reader < 0);
  std::vector<int> roles = {
      plan.video_consumers[0], plan.video_consumers[1],
      plan.video_consumers[2], plan.video_producers[0],
      plan.video_producers[1], plan.video_producers[2], plan.filler};
  CHECK(std::set<int>(roles.begin(), roles.end()).size() == roles.size());
  CHECK(plan.video_consumers[0] != 0);
  CHECK(plan.video_consumers[0] != 1);
  for (std::size_t left = 0; left < roles.size(); ++left) {
    for (std::size_t right = left + 1; right < roles.size(); ++right) {
      CHECK(!shares_a_core(cores, roles[left], roles[right]));
    }
  }
  for (int cpu : plan.shadow) {
    CHECK(cpu >= 0);
    for (int role : roles) {
      CHECK(cpu != role);
      CHECK(!shares_a_core(cores, cpu, role));
    }
  }
  CHECK(std::set<int>(plan.shadow.begin(), plan.shadow.end()).size() ==
        plan.shadow.size());
}

TEST_CASE(send_affinity_leaves_shadow_unpinned_when_no_spare_cores) {
  std::vector<std::vector<int>> cores;
  for (int index = 0; index < 8; ++index) cores.push_back({index});
  const auto plan = joint_sensor_orchestrator::plan_send_affinity(cores);
  CHECK(plan.enabled);
  CHECK(plan.filler == 7);
  for (int cpu : plan.shadow) CHECK(cpu < 0);
}

TEST_CASE(send_affinity_stays_off_when_there_are_not_enough_cores) {
  std::vector<std::vector<int>> cores;
  for (int index = 0; index < 7; ++index) cores.push_back({index});
  const auto plan = joint_sensor_orchestrator::plan_send_affinity(cores);
  CHECK(!plan.enabled);
}
