#pragma once

#include "joint_sensor_player/player.hpp"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

namespace joint_sensor_orchestrator {

struct StartAnchors {
  std::uint64_t play_start_ns{};
  std::uint64_t utc_anchor_us{};
};

using ThreadFactory =
    std::function<std::thread(std::function<void()>)>;

class StartGate {
 public:
  explicit StartGate(std::size_t expected_workers);

  bool arrive_and_wait(
      const joint_sensor_player::CancellationToken& cancellation,
      StartAnchors& anchors);
  bool wait_until_ready(std::chrono::milliseconds timeout);
  void release(StartAnchors anchors);
  void cancel() noexcept;

  std::size_t ready_count() const noexcept;
  bool released() const noexcept;

 private:
  const std::size_t expected_workers_;
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  std::size_t ready_workers_{};
  bool released_{};
  bool cancelled_{};
  StartAnchors anchors_;
};

}  // namespace joint_sensor_orchestrator
