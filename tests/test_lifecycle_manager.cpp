#include "joint_sensor_orchestrator/lifecycle_manager.hpp"
#include "test_framework.hpp"

#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

using joint_sensor_orchestrator::LifecycleManager;
using joint_sensor_orchestrator::LifecycleState;

TEST_CASE(lifecycle_requires_approved_transition_order) {
  LifecycleManager manager;
  CHECK(manager.state() == LifecycleState::Created);
  CHECK_THROWS(manager.transition(LifecycleState::Ready));

  manager.transition(LifecycleState::Preparing);
  manager.transition(LifecycleState::Ready);
  CHECK_THROWS(manager.transition(LifecycleState::Running));
  manager.transition(LifecycleState::Armed);
  manager.transition(LifecycleState::Running);
  manager.transition(LifecycleState::Stopping);
  manager.transition(LifecycleState::Stopped);
  CHECK(manager.state() == LifecycleState::Stopped);
}

TEST_CASE(lifecycle_preserves_first_failure_and_requests_shared_stop) {
  LifecycleManager manager;
  manager.transition(LifecycleState::Preparing);
  const auto shared_token = manager.stop_token();

  manager.fail("video", "short write");
  manager.fail("radar", "cancelled");

  CHECK(manager.state() == LifecycleState::Failed);
  CHECK(manager.failure().component == "video");
  CHECK(manager.failure().reason == "short write");
  CHECK(manager.additional_failures().size() == 1);
  CHECK(manager.additional_failures()[0].component == "radar");
  CHECK(shared_token == manager.stop_token());
  CHECK(shared_token->stop_requested());

  manager.transition(LifecycleState::Stopping);
  manager.transition(LifecycleState::Stopped);
}

TEST_CASE(lifecycle_concurrent_fail_preserves_one_root_and_records_all) {
  constexpr int kFailureCount = 16;
  LifecycleManager manager;
  manager.transition(LifecycleState::Preparing);

  std::vector<std::thread> workers;
  workers.reserve(kFailureCount);
  for (int index = 0; index < kFailureCount; ++index) {
    workers.emplace_back([&manager, index] {
      manager.fail("component-" + std::to_string(index),
                   "reason-" + std::to_string(index));
    });
  }
  for (auto& worker : workers) worker.join();

  const auto snapshot = manager.snapshot();
  CHECK(snapshot.state == LifecycleState::Failed);
  CHECK(!snapshot.first_failure.component.empty());
  CHECK(snapshot.additional_failures.size() == kFailureCount - 1);
  CHECK(manager.stop_token()->stop_requested());

  std::set<std::string> components;
  components.insert(snapshot.first_failure.component);
  for (const auto& failure : snapshot.additional_failures) {
    components.insert(failure.component);
  }
  CHECK(components.size() == kFailureCount);
  for (int index = 0; index < kFailureCount; ++index) {
    CHECK(components.count("component-" + std::to_string(index)) == 1);
  }
}

TEST_CASE(lifecycle_request_stop_is_idempotent) {
  LifecycleManager manager;
  const auto token = manager.stop_token();
  manager.request_stop();
  manager.request_stop();
  CHECK(token->stop_requested());
  CHECK(manager.state() == LifecycleState::Created);
}

TEST_CASE(lifecycle_snapshot_contains_state_history_and_failures) {
  LifecycleManager manager;
  manager.transition(LifecycleState::Preparing);
  manager.fail("cache", "corrupt");
  const auto snapshot = manager.snapshot();

  CHECK(snapshot.state == LifecycleState::Failed);
  CHECK(snapshot.transitions.size() == 3);
  CHECK(snapshot.transitions[0] == LifecycleState::Created);
  CHECK(snapshot.transitions[2] == LifecycleState::Failed);
  CHECK(snapshot.first_failure.component == "cache");
  CHECK(snapshot.additional_failures.empty());
}

TEST_CASE(lifecycle_rejects_failure_without_diagnostic) {
  LifecycleManager manager;
  CHECK_THROWS(manager.fail("", "reason"));
  CHECK_THROWS(manager.fail("video", ""));
  CHECK(!manager.stop_token()->stop_requested());
}
