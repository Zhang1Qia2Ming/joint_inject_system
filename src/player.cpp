#include "joint_sensor_player/player.hpp"

#include "joint_sensor_player/packet_sequence.hpp"
#include "joint_sensor_player/packet_time_rewriter.hpp"
#include "joint_sensor_player/report.hpp"
#include "joint_sensor_player/timeline.hpp"

#include <time.h>

#include <cerrno>
#include <limits>
#include <queue>
#include <stdexcept>
#include <vector>

namespace joint_sensor_player {
namespace {

struct Cursor {
  const LidarChannelConfig* channel{};
  std::vector<LidarFrame>* frames{};
  std::size_t channel_order{};
  std::size_t frame_index{};
  std::size_t packet_index{};
  std::uint64_t deadline_ns{};
};

struct LaterDeadline {
  bool operator()(const Cursor& left, const Cursor& right) const {
    if (left.deadline_ns != right.deadline_ns) {
      return left.deadline_ns > right.deadline_ns;
    }
    return left.channel_order > right.channel_order;
  }
};

bool set_first_frame(Cursor& cursor, const PlaybackWindow& window,
                     std::uint64_t play_start_ns) {
  while (cursor.frame_index < cursor.frames->size() &&
         (*cursor.frames)[cursor.frame_index].source_timestamp_ns <
             window.source_start_ns) {
    ++cursor.frame_index;
  }
  if (cursor.frame_index >= cursor.frames->size() ||
      (*cursor.frames)[cursor.frame_index].source_timestamp_ns >
          window.source_end_ns) {
    return false;
  }
  cursor.packet_index = 0;
  cursor.deadline_ns = packet_deadline_ns(
      play_start_ns, window.source_start_ns,
      (*cursor.frames)[cursor.frame_index].source_timestamp_ns, 0,
      cursor.channel->packet_interval_ns, cursor.channel->time_offset_us);
  return true;
}

bool advance(Cursor& cursor, const PlaybackWindow& window,
             std::uint64_t play_start_ns, bool& completed_frame) {
  completed_frame = false;
  const auto& frame = (*cursor.frames)[cursor.frame_index];
  ++cursor.packet_index;
  if (cursor.packet_index >= frame.packets.size()) {
    completed_frame = true;
    ++cursor.frame_index;
    cursor.packet_index = 0;
    if (cursor.frame_index >= cursor.frames->size() ||
        (*cursor.frames)[cursor.frame_index].source_timestamp_ns >
            window.source_end_ns) {
      return false;
    }
  }
  cursor.deadline_ns = packet_deadline_ns(
      play_start_ns, window.source_start_ns,
      (*cursor.frames)[cursor.frame_index].source_timestamp_ns,
      cursor.packet_index, cursor.channel->packet_interval_ns,
      cursor.channel->time_offset_us);
  return true;
}

}  // namespace

std::uint64_t MonotonicClock::now_ns() const {
  timespec value{};
  if (::clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    throw std::runtime_error("clock_gettime失败");
  }
  return static_cast<std::uint64_t>(value.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
}

bool MonotonicClock::sleep_until_ns(std::uint64_t deadline_ns) {
  timespec deadline{};
  deadline.tv_sec = static_cast<time_t>(deadline_ns / 1000000000ULL);
  deadline.tv_nsec = static_cast<long>(deadline_ns % 1000000000ULL);
  int result = 0;
  do {
    result = ::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline,
                               nullptr);
  } while (result == EINTR);
  return result == 0;
}

WaitStatus cancellation_aware_sleep_until(
    Clock& clock, std::uint64_t deadline_ns,
    const CancellationToken* cancellation, std::uint64_t max_step_ns) {
  if (max_step_ns == 0) {
    throw std::invalid_argument("取消感知等待步长必须大于零");
  }
  while (true) {
    if (cancellation != nullptr && cancellation->stop_requested()) {
      return WaitStatus::Cancelled;
    }
    const auto now_ns = clock.now_ns();
    if (now_ns >= deadline_ns) return WaitStatus::Reached;
    const auto remaining = deadline_ns - now_ns;
    const auto step = std::min(remaining, max_step_ns);
    if (!clock.sleep_until_ns(now_ns + step)) return WaitStatus::Error;
  }
}

PlaybackResult play_catalog(const PlayerConfig& config,
                            McapCatalog& catalog, Clock& clock,
                            DatagramSink& sink, AsyncReport* report,
                            CancellationToken* cancellation,
                            const LidarPacketRewriter* rewriter,
                            std::optional<std::uint64_t>
                                explicit_play_start_ns,
                            FrameStatObserver frame_observer) {
  PlaybackResult result;
  try {
    const auto window = build_playback_window(
        catalog, config.channels, config.allow_partial_start);
    const bool looping = config.loop_count == 0 || config.loop_count > 1;
    std::uint64_t cycle_period_ns = 0;
    SequencePlans sequence_plans;
    if (looping) {
      const auto frame_period =
          nominal_frame_period_ns(catalog, config.channels);
      const auto source_duration =
          window.source_end_ns - window.source_start_ns;
      if (source_duration >
          std::numeric_limits<std::uint64_t>::max() - frame_period) {
        throw std::runtime_error("循环周期溢出");
      }
      cycle_period_ns = source_duration + frame_period;
      sequence_plans = build_sequence_plans(
          catalog, config.channels, config.sequence_mode,
          window.source_start_ns, window.source_end_ns);
    }
    const auto now_ns = clock.now_ns();
    if (explicit_play_start_ns.has_value()) {
      if (*explicit_play_start_ns < now_ns) {
        throw std::runtime_error("显式播放起点早于当前单调时间");
      }
      result.play_start_ns = *explicit_play_start_ns;
    } else {
      const auto start_delay_ns =
          static_cast<std::uint64_t>(config.start_delay_ms) * 1000000ULL;
      if (now_ns >
          std::numeric_limits<std::uint64_t>::max() - start_delay_ns) {
        throw std::runtime_error("播放起始期限溢出");
      }
      result.play_start_ns = now_ns + start_delay_ns;
    }

    for (std::uint64_t cycle = 0;
         config.loop_count == 0 || cycle < config.loop_count; ++cycle) {
      if (cancellation != nullptr && cancellation->stop_requested()) {
        result.ok = true;
        result.cancelled = true;
        result.stop_reason = "cancelled";
        return result;
      }
      if (cycle_period_ns != 0 &&
          cycle > (std::numeric_limits<std::uint64_t>::max() -
                   result.play_start_ns) /
                      cycle_period_ns) {
        throw std::runtime_error("循环起点溢出");
      }
      const auto cycle_start_ns =
          result.play_start_ns + cycle * cycle_period_ns;
      std::priority_queue<Cursor, std::vector<Cursor>, LaterDeadline> queue;
      for (std::size_t index = 0; index < config.channels.size(); ++index) {
        const auto& channel = config.channels[index];
        if (!channel.enabled) continue;
        auto found = catalog.frames.find(channel.topic);
        if (found == catalog.frames.end() || found->second.empty()) continue;
        Cursor cursor{&channel, &found->second, index, 0, 0, 0};
        if (set_first_frame(cursor, window, cycle_start_ns)) {
          queue.push(cursor);
        }
      }

      while (!queue.empty()) {
        if (cancellation != nullptr && cancellation->stop_requested()) {
          result.ok = true;
          result.cancelled = true;
          result.stop_reason = "cancelled";
          return result;
        }
        auto cursor = queue.top();
        queue.pop();
        auto& frame = (*cursor.frames)[cursor.frame_index];
        auto& packet = frame.packets[cursor.packet_index];
        if (rewriter == nullptr && cycle > 0 &&
            config.sequence_mode == SequenceMode::Continuous) {
          const auto plan = sequence_plans.find(cursor.channel->topic);
          if (plan == sequence_plans.end()) {
            throw std::runtime_error("缺少雷达UDP序号计划: " +
                                     cursor.channel->topic);
          }
          advance_packet_sequence(packet, plan->second);
        }
        std::vector<std::uint8_t> rewritten;
        const std::vector<std::uint8_t>* send_packet = &packet;
        if (rewriter != nullptr) {
          rewritten = rewriter->rewrite(cursor.channel->topic, packet, cycle);
          send_packet = &rewritten;
        }
        const auto wait = cancellation_aware_sleep_until(
            clock, cursor.deadline_ns, cancellation);
        if (wait == WaitStatus::Cancelled) {
          result.ok = true;
          result.cancelled = true;
          result.stop_reason = "cancelled";
          return result;
        }
        if (wait == WaitStatus::Error) {
          result.error = "等待绝对发送期限失败";
          result.stop_reason = result.error;
          return result;
        }
        if (cancellation != nullptr && cancellation->stop_requested()) {
          result.ok = true;
          result.cancelled = true;
          result.stop_reason = "cancelled";
          return result;
        }
        const auto before_send_ns = clock.now_ns();
        if (before_send_ns > cursor.deadline_ns) {
          result.max_lateness_ns =
              std::max(result.max_lateness_ns,
                       before_send_ns - cursor.deadline_ns);
          if (before_send_ns - cursor.deadline_ns >
              static_cast<std::uint64_t>(config.max_lateness_us) * 1000ULL) {
            result.error =
                "发送迟到超过门限: slot=" +
                std::to_string(cursor.channel->slot) +
                " lateness_us=" +
                std::to_string((before_send_ns - cursor.deadline_ns) / 1000);
            result.stop_reason = result.error;
            return result;
          }
        }
        const auto frame_timestamp = frame.source_timestamp_ns;
        const auto frame_packet_count =
            static_cast<std::uint32_t>(frame.packets.size());
        const auto sent_deadline_ns = cursor.deadline_ns;
        std::string error;
        if (!sink.send(*cursor.channel, *send_packet, cursor.deadline_ns,
                       error)) {
          result.error = error;
          result.stop_reason = result.error;
          return result;
        }
        const auto after_send_ns = clock.now_ns();
        if (after_send_ns > cursor.deadline_ns) {
          result.max_lateness_ns =
              std::max(result.max_lateness_ns,
                       after_send_ns - cursor.deadline_ns);
        }
        ++result.packets_sent;
        if (after_send_ns > cursor.deadline_ns &&
            after_send_ns - cursor.deadline_ns >
                static_cast<std::uint64_t>(config.max_lateness_us) *
                    1000ULL) {
          result.error =
              "发送完成迟到超过门限: slot=" +
              std::to_string(cursor.channel->slot) +
              " lateness_us=" +
              std::to_string((after_send_ns - cursor.deadline_ns) / 1000);
          result.stop_reason = result.error;
          return result;
        }
        bool completed_frame = false;
        if (advance(cursor, window, cycle_start_ns, completed_frame)) {
          queue.push(cursor);
        }
        if (completed_frame) {
          ++result.frames_sent;
          const auto lateness = after_send_ns > sent_deadline_ns
                                    ? after_send_ns - sent_deadline_ns
                                    : 0;
          const FrameStat stat{
              cursor.channel->slot, cursor.channel->topic, frame_timestamp,
              sent_deadline_ns, after_send_ns, frame_packet_count, lateness};
          if (report != nullptr) {
            (void)report->try_submit(
                stat);
          }
          if (frame_observer) frame_observer(stat);
        }
      }
      ++result.loops_completed;
    }
    result.ok = true;
    result.stop_reason = "completed";
  } catch (const std::exception& error) {
    result.error = error.what();
    result.stop_reason = result.error;
  }
  return result;
}

}  // namespace joint_sensor_player
