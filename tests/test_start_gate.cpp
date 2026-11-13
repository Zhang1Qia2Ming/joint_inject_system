#include "joint_sensor_orchestrator/start_gate.hpp"
#include "test_framework.hpp"

#include <atomic>
#include <chrono>
#include <thread>

TEST_CASE(start_gate_releases_all_ready_workers_with_identical_anchors) {
  joint_sensor_orchestrator::StartGate gate(2);
  joint_sensor_player::CancellationToken cancellation;
  std::array<joint_sensor_orchestrator::StartAnchors, 2> observed;
  std::atomic<unsigned> crossed{};
  std::thread first([&] {
    if (gate.arrive_and_wait(cancellation, observed[0])) ++crossed;
  });
  std::thread second([&] {
    if (gate.arrive_and_wait(cancellation, observed[1])) ++crossed;
  });

  CHECK(gate.wait_until_ready(std::chrono::milliseconds(100)));
  CHECK(gate.ready_count() == 2);
  CHECK(crossed.load() == 0);
  gate.release({10'000, 2'000'000});
  first.join();
  second.join();

  CHECK(crossed.load() == 2);
  CHECK(observed[0].play_start_ns == 10'000);
  CHECK(observed[1].play_start_ns == 10'000);
  CHECK(observed[0].utc_anchor_us == 2'000'000);
  CHECK(observed[1].utc_anchor_us == 2'000'000);
}

TEST_CASE(start_gate_cancellation_wakes_ready_workers_without_release) {
  joint_sensor_orchestrator::StartGate gate(1);
  joint_sensor_player::CancellationToken cancellation;
  joint_sensor_orchestrator::StartAnchors ignored;
  std::atomic<bool> crossed{};
  std::thread worker([&] {
    crossed = gate.arrive_and_wait(cancellation, ignored);
  });

  CHECK(gate.wait_until_ready(std::chrono::milliseconds(100)));
  cancellation.request_stop();
  gate.cancel();
  worker.join();

  CHECK(!crossed.load());
}
