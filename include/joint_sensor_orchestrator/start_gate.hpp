#pragma once

// 开播对齐点。每个参加的线程先到达，主线程等人数齐了再放出锚点。
// 放出之后各线程拿到同一个 play_start，第一帧的期限都从它往后算。
// 取消时已经在等的线程立刻返回，不再发送。

#include "joint_sensor_player/player.hpp"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

namespace joint_sensor_orchestrator {

// 一次开播共用的两个时间原点。
// play_start_ns 是单调时钟，视频槽和雷达包的期限都从它加上源时间偏移得到。
// utc_anchor_us 是墙上时钟，用来改写雷达包里的传感器时间。
struct StartAnchors {
  std::uint64_t play_start_ns{};
  std::uint64_t utc_anchor_us{};
};

using ThreadFactory =
    std::function<std::thread(std::function<void()>)>;

class StartGate {
 public:
  explicit StartGate(std::size_t expected_workers);

  // 工作线程调用。人数到齐并被 release 之后，anchors 里就是本次开播原点。
  bool arrive_and_wait(
      const joint_sensor_player::CancellationToken& cancellation,
      StartAnchors& anchors);
  // 主线程调用。超时返回 false，调用方负责停掉已经起来的线程。
  bool wait_until_ready(std::chrono::milliseconds timeout);
  // 把锚点交给所有已经到达的线程，它们从这里同时往下走。
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
