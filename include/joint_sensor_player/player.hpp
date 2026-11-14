#pragma once

// 雷达播放器的运行时类型：停止开关、单调时钟、一次播放的结果。
// 联合播放和单独的雷达播放器都用这套。睡到绝对期限时会分段醒来，
// 这样停止请求不用等一整拍。

#include "joint_sensor_player/config.hpp"
#include "joint_sensor_player/mcap.hpp"
#include "joint_sensor_player/report.hpp"
#include "joint_sensor_player/udp_sender.hpp"

#include <atomic>
#include <cstddef>
#include <csignal>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

namespace joint_sensor_player {

class AsyncReport;
class LidarPacketRewriter;
using FrameStatObserver = std::function<void(const FrameStat&)>;

class CancellationToken {
 public:
  explicit CancellationToken(
      const volatile std::sig_atomic_t* signal_stop = nullptr) noexcept
      : signal_stop_(signal_stop) {}

  void request_stop() noexcept { stop_.store(true, std::memory_order_relaxed); }
  bool stop_requested() const noexcept {
    return stop_.load(std::memory_order_relaxed) ||
           (signal_stop_ != nullptr && *signal_stop_ != 0);
  }

 private:
  std::atomic<bool> stop_{false};
  const volatile std::sig_atomic_t* signal_stop_{};
};

class Clock {
 public:
  virtual ~Clock() = default;
  virtual std::uint64_t now_ns() const = 0;
  virtual bool sleep_until_ns(std::uint64_t deadline_ns) = 0;
};

class MonotonicClock final : public Clock {
 public:
  std::uint64_t now_ns() const override;
  bool sleep_until_ns(std::uint64_t deadline_ns) override;
};

enum class WaitStatus {
  Reached,
  Cancelled,
  Error,
};

// 睡到 deadline_ns，或者停止请求先到。每步最多 max_step_ns，默认 1 毫秒。
WaitStatus cancellation_aware_sleep_until(
    Clock& clock, std::uint64_t deadline_ns,
    const CancellationToken* cancellation,
    std::uint64_t max_step_ns = 1'000'000ULL);

struct PlaybackResult {
  bool ok{};
  std::uint64_t packets_sent{};
  std::uint64_t frames_sent{};
  std::size_t max_buffered_packets{};
  std::uint64_t max_lateness_ns{};
  std::uint64_t play_start_ns{};
  std::uint64_t loops_completed{};
  bool cancelled{};
  std::string stop_reason;
  std::string error;
};

// 把已经扫进内存的目录按包的绝对期限发出去。联合播放的长包不走这里，
// 走 RadarEngine 的流式发送，避免整份点云都驻留。
PlaybackResult play_catalog(const PlayerConfig& config,
                            McapCatalog& catalog, Clock& clock,
                            DatagramSink& sink, AsyncReport* report = nullptr,
                            CancellationToken* cancellation = nullptr,
                            const LidarPacketRewriter* rewriter = nullptr,
                            std::optional<std::uint64_t>
                                explicit_play_start_ns = std::nullopt,
                            FrameStatObserver frame_observer = {});

}  // namespace joint_sensor_player
