// 八路雷达的发送实现。worker_main 参加 StartGate，流式读盘的线程不参加。
// 读盘线程把已到安全水位的包按通道交给发送线程。每路自己睡到期限，
// 用醒来时刻判断迟到，然后才 sendmsg。一路的 sendmsg 堵住时，其它路继续发。

#include "joint_sensor_orchestrator/radar_engine.hpp"
#include "joint_sensor_orchestrator/thread_affinity.hpp"

#include "joint_sensor_player/packet_time_rewriter.hpp"
#include "joint_sensor_player/ptc_server.hpp"
#include "joint_sensor_player/timeline.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <limits>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

namespace joint_sensor_orchestrator {
namespace {

std::vector<std::uint8_t> read_binary(
    const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("无法读取PTC标定文件: " + path.string());
  }
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

class ExistingPtcService final : public RadarPtcService {
 public:
  explicit ExistingPtcService(
      const joint_sensor_player::PlayerConfig& config) {
    std::vector<joint_sensor_player::PtcEndpoint> endpoints;
    for (const auto& channel : config.channels) {
      if (!channel.enabled) continue;
      endpoints.push_back(
          {channel.source_ip, config.ptc_port,
           read_binary(channel.correction_file)});
    }
    service_ =
        std::make_unique<joint_sensor_player::PtcCalibrationService>(
            std::move(endpoints));
  }

  void start() override { service_->start(); }
  void stop() noexcept override { service_.reset(); }

 private:
  std::unique_ptr<joint_sensor_player::PtcCalibrationService> service_;
};

std::uint64_t checked_add(std::uint64_t left, std::uint64_t right,
                          const char* message) {
  if (left > std::numeric_limits<std::uint64_t>::max() - right) {
    throw std::overflow_error(message);
  }
  return left + right;
}

}  // namespace

std::uint64_t earliest_possible_frame_deadline_ns(
    const joint_sensor_player::PlayerConfig& config,
    std::uint64_t common_source_start_ns,
    std::uint64_t frame_bag_log_time_ns,
    std::uint64_t play_start_ns) {
  std::optional<std::uint64_t> earliest;
  for (const auto& channel : config.channels) {
    if (!channel.enabled) continue;
    const auto deadline = joint_sensor_player::packet_deadline_ns(
        play_start_ns, common_source_start_ns,
        frame_bag_log_time_ns, 0, channel.packet_interval_ns,
        channel.time_offset_us);
    earliest = earliest ? std::min(*earliest, deadline) : deadline;
  }
  if (!earliest) {
    throw std::runtime_error("无法计算未来雷达帧最早deadline");
  }
  return *earliest;
}

bool packet_deadline_is_safe(
    std::uint64_t packet_deadline_ns,
    std::uint64_t unread_frame_deadline_lower_bound_ns) noexcept {
  return packet_deadline_ns <
         unread_frame_deadline_lower_bound_ns;
}

StreamingRadarPreflight analyze_streaming_radar(
    const joint_sensor_player::PlayerConfig& config,
    const joint_sensor_player::LightweightMcapCatalog& catalog,
    SourceWindow common_window) {
  if (common_window.start_ns > common_window.end_ns) {
    throw std::invalid_argument("流式雷达预检共同窗口无效");
  }
  struct PlannedFrame {
    const joint_sensor_player::LidarChannelConfig* channel{};
    std::uint32_t packet_count{};
  };
  std::map<std::uint64_t, std::vector<PlannedFrame>> groups;
  const auto offset_ns = [](std::int64_t offset_us) {
    if (offset_us >
            std::numeric_limits<std::int64_t>::max() / 1000 ||
        offset_us <
            std::numeric_limits<std::int64_t>::min() / 1000) {
      throw std::overflow_error("流式雷达通道time_offset溢出");
    }
    return offset_us * 1000;
  };
  std::optional<std::int64_t> minimum_offset_ns;
  for (const auto& channel : config.channels) {
    if (!channel.enabled) continue;
    const auto channel_offset_ns = offset_ns(channel.time_offset_us);
    minimum_offset_ns =
        minimum_offset_ns
            ? std::min(*minimum_offset_ns, channel_offset_ns)
            : channel_offset_ns;
  }
  if (!minimum_offset_ns) {
    throw std::runtime_error("流式雷达预检没有启用通道");
  }

  for (const auto& channel : config.channels) {
    if (!channel.enabled) continue;
    const auto found = catalog.frames.find(channel.topic);
    if (found == catalog.frames.end()) {
      throw std::runtime_error("流式雷达预检缺少topic: " +
                               channel.topic);
    }
    bool selected = false;
    for (const auto& frame : found->second) {
      if (frame.bag_log_time_ns < common_window.start_ns ||
          frame.bag_log_time_ns > common_window.end_ns) {
        continue;
      }
      if (frame.packet_count == 0) {
        throw std::runtime_error("流式雷达预检发现空帧: " +
                                 channel.topic);
      }
      selected = true;
      groups[frame.bag_log_time_ns].push_back(
          {&channel, frame.packet_count});
    }
    if (!selected) {
      throw std::runtime_error("共同窗口内没有雷达topic数据: " +
                               channel.topic);
    }
  }

  const auto relative_deadline =
      [&](const PlannedFrame& frame, std::uint64_t bag_log_time_ns,
          std::uint32_t packet_index) {
        const auto channel_offset_ns =
            offset_ns(frame.channel->time_offset_us);
        std::uint64_t adjusted_offset{};
        if (*minimum_offset_ns >= 0 || channel_offset_ns < 0) {
          adjusted_offset = static_cast<std::uint64_t>(
              channel_offset_ns - *minimum_offset_ns);
        } else {
          const auto minimum_magnitude =
              static_cast<std::uint64_t>(
                  -(*minimum_offset_ns + 1)) +
              1;
          adjusted_offset =
              static_cast<std::uint64_t>(channel_offset_ns) +
              minimum_magnitude;
        }
        auto deadline =
            checked_add(bag_log_time_ns - common_window.start_ns,
                        adjusted_offset,
                        "流式雷达相对deadline溢出");
        if (packet_index != 0 &&
            frame.channel->packet_interval_ns >
                std::numeric_limits<std::uint64_t>::max() /
                    packet_index) {
          throw std::overflow_error("流式雷达包间隔乘法溢出");
        }
        deadline = checked_add(
            deadline,
            static_cast<std::uint64_t>(packet_index) *
                frame.channel->packet_interval_ns,
            "流式雷达相对deadline溢出");
        return deadline;
      };

  std::priority_queue<std::uint64_t,
                      std::vector<std::uint64_t>,
                      std::greater<std::uint64_t>>
      heap;
  StreamingRadarPreflight result;
  result.required_frame_reorder = catalog.required_reorder_frames;
  for (auto group = groups.begin(); group != groups.end(); ++group) {
    std::uint64_t staged_min =
        std::numeric_limits<std::uint64_t>::max();
    std::size_t staged_packets = 0;
    for (const auto& frame : group->second) {
      staged_min =
          std::min(staged_min,
                   relative_deadline(frame, group->first, 0));
      if (staged_packets >
          std::numeric_limits<std::size_t>::max() -
              frame.packet_count) {
        throw std::overflow_error("流式雷达packet计数溢出");
      }
      staged_packets += frame.packet_count;
    }
    const auto next = std::next(group);
    const auto future_lower_bound =
        next == groups.end()
            ? std::numeric_limits<std::uint64_t>::max()
            : next->first - common_window.start_ns;
    const auto safe_bound =
        std::min(staged_min, future_lower_bound);
    while (!heap.empty() &&
           packet_deadline_is_safe(heap.top(), safe_bound)) {
      heap.pop();
    }
    if (heap.size() >
        std::numeric_limits<std::size_t>::max() -
            staged_packets) {
      throw std::overflow_error("流式雷达heap计数溢出");
    }
    result.required_packet_buffer =
        std::max(result.required_packet_buffer,
                 heap.size() + staged_packets);
    for (const auto& frame : group->second) {
      for (std::uint32_t packet = 0;
           packet < frame.packet_count; ++packet) {
        heap.push(relative_deadline(
            frame, group->first, packet));
      }
    }
    while (!heap.empty() &&
           packet_deadline_is_safe(
               heap.top(), future_lower_bound)) {
      heap.pop();
    }
  }
  return result;
}

joint_sensor_player::PlaybackResult play_streaming_radar(
    const joint_sensor_player::PlayerConfig& config,
    const joint_sensor_player::LightweightMcapCatalog& catalog,
    SourceWindow common_window, joint_sensor_player::Clock& clock,
    joint_sensor_player::DatagramSink& sink,
    joint_sensor_player::CancellationToken& cancellation,
    std::uint64_t utc_anchor_us, std::uint64_t play_start_ns,
    const RadarFrameStreamFunction& stream,
    joint_sensor_player::FrameStatObserver frame_observer,
    StreamingRadarLimits limits, int reader_cpu) {
  joint_sensor_player::PlaybackResult result;
  result.play_start_ns = play_start_ns;
  if (cancellation.stop_requested()) {
    result.ok = true;
    result.cancelled = true;
    result.stop_reason = "cancelled";
    return result;
  }
  try {
    if (!stream || limits.packet_queue_capacity == 0 ||
        limits.frame_reorder_capacity == 0 ||
        common_window.start_ns > common_window.end_ns) {
      throw std::invalid_argument("流式雷达参数或缓冲上限无效");
    }
    const auto preflight =
        analyze_streaming_radar(config, catalog, common_window);
    if (preflight.required_packet_buffer >
        limits.packet_queue_capacity) {
      throw std::runtime_error(
          "雷达deadline重叠所需packet缓冲超过明确上限: required=" +
          std::to_string(preflight.required_packet_buffer) +
          " limit=" +
          std::to_string(limits.packet_queue_capacity));
    }
    std::unordered_map<std::string,
                       const joint_sensor_player::LidarChannelConfig*>
        channels;
    std::uint64_t radar_start = std::numeric_limits<std::uint64_t>::max();
    std::optional<std::uint64_t> source_packet_origin;
    std::map<std::uint64_t, std::size_t> packets_per_bag_time;
    for (const auto& channel : config.channels) {
      if (!channel.enabled) continue;
      channels.emplace(channel.topic, &channel);
      const auto found = catalog.frames.find(channel.topic);
      if (found == catalog.frames.end()) {
        throw std::runtime_error("流式雷达预检缺少topic: " + channel.topic);
      }
      bool selected = false;
      for (const auto& frame : found->second) {
        if (frame.bag_log_time_ns < common_window.start_ns ||
            frame.bag_log_time_ns > common_window.end_ns) {
          continue;
        }
        selected = true;
        radar_start = std::min(radar_start, frame.bag_log_time_ns);
        source_packet_origin =
            source_packet_origin
                ? std::min(*source_packet_origin,
                           frame.first_packet_time_us)
                : frame.first_packet_time_us;
        auto& packet_count = packets_per_bag_time[frame.bag_log_time_ns];
        if (frame.packet_count > limits.packet_queue_capacity ||
            packet_count >
                limits.packet_queue_capacity - frame.packet_count) {
          throw std::runtime_error(
              "同一bag_log_time雷达包数超过流式队列上限: " +
              std::to_string(limits.packet_queue_capacity));
        }
        packet_count += frame.packet_count;
      }
      if (!selected) {
        throw std::runtime_error("共同窗口内没有雷达topic数据: " +
                                 channel.topic);
      }
    }
    if (!source_packet_origin ||
        radar_start == std::numeric_limits<std::uint64_t>::max()) {
      throw std::runtime_error("共同窗口内没有流式雷达数据");
    }
    const auto source_offset_ns = radar_start - common_window.start_ns;
    const auto radar_anchor_us =
        checked_add(utc_anchor_us, source_offset_ns / 1000ULL,
                    "流式雷达UTC锚点溢出");

    struct ScheduledPacket {
      const joint_sensor_player::LidarChannelConfig* channel{};
      std::string topic;
      std::vector<std::uint8_t> payload;
      std::uint64_t bag_log_time_ns{};
      std::uint64_t source_time_ns{};
      std::uint64_t deadline_ns{};
      std::uint32_t frame_packet_count{};
      std::uint64_t frame_order{};
      std::uint32_t packet_index{};
      std::size_t channel_order{};
    };
    struct LaterPacket {
      bool operator()(const ScheduledPacket& left,
                      const ScheduledPacket& right) const {
        if (left.deadline_ns != right.deadline_ns) {
          return left.deadline_ns > right.deadline_ns;
        }
        if (left.channel_order != right.channel_order) {
          return left.channel_order > right.channel_order;
        }
        if (left.frame_order != right.frame_order) {
          return left.frame_order > right.frame_order;
        }
        return left.packet_index > right.packet_index;
      }
    };
    struct Shared {
      std::mutex mutex;
      std::condition_variable changed;
      std::priority_queue<ScheduledPacket,
                          std::vector<ScheduledPacket>, LaterPacket>
          packets;
      std::optional<std::uint64_t> unread_deadline_lower_bound;
      std::size_t max_packets{};
      bool producer_done{};
      std::string producer_error;
    } shared;

    std::thread producer([&] {
      pin_current_thread(reader_cpu);
      try {
        std::optional<std::uint64_t> current_bag_time;
        std::vector<ScheduledPacket> current_bag_packets;
        std::unordered_map<std::string, std::uint64_t>
            previous_packet_time;
        std::unordered_map<std::string, std::uint32_t>
            previous_packet_sequence;
        std::unordered_map<std::string, std::uint64_t>
            next_frame_order;

        const auto enqueue_group =
            [&](std::vector<ScheduledPacket>& group,
                std::uint64_t future_lower_bound) {
              if (group.empty()) return;
              const auto staged_lower_bound =
                  std::min_element(
                      group.begin(), group.end(),
                      [](const auto& left, const auto& right) {
                        return left.deadline_ns < right.deadline_ns;
                      })
                      ->deadline_ns;
              std::unique_lock<std::mutex> lock(shared.mutex);
              shared.unread_deadline_lower_bound =
                  std::min(staged_lower_bound, future_lower_bound);
              shared.changed.notify_all();
              while (shared.packets.size() + group.size() >
                     limits.packet_queue_capacity) {
                if (cancellation.stop_requested()) return;
                if (shared.packets.empty() ||
                    !packet_deadline_is_safe(
                        shared.packets.top().deadline_ns,
                        *shared.unread_deadline_lower_bound)) {
                  throw std::runtime_error(
                      "雷达deadline重叠所需packet缓冲超过明确上限: " +
                      std::to_string(limits.packet_queue_capacity));
                }
                shared.changed.wait_for(
                    lock, std::chrono::milliseconds(1));
              }
              for (auto& packet : group) {
                shared.packets.push(std::move(packet));
              }
              group.clear();
              shared.max_packets =
                  std::max(shared.max_packets, shared.packets.size());
              shared.unread_deadline_lower_bound = future_lower_bound;
              lock.unlock();
              shared.changed.notify_all();
            };
        const auto enqueue_completed_bag_time = [&]() {
          if (!current_bag_time || current_bag_packets.empty()) return;
          const auto expected =
              packets_per_bag_time.at(*current_bag_time);
          if (current_bag_packets.size() < expected) return;
          if (current_bag_packets.size() > expected) {
            throw std::runtime_error(
                "流式雷达同一bag_log_time包数超过轻量预检");
          }
          const auto next =
              packets_per_bag_time.upper_bound(*current_bag_time);
          const auto future_lower_bound =
              next == packets_per_bag_time.end()
                  ? std::numeric_limits<std::uint64_t>::max()
                  : earliest_possible_frame_deadline_ns(
                        config, common_window.start_ns, next->first,
                        play_start_ns);
          enqueue_group(current_bag_packets, future_lower_bound);
        };

        (void)stream([&](const std::string& topic,
                         joint_sensor_player::LidarFrame frame) {
          if (cancellation.stop_requested()) return false;
          const auto channel_it = channels.find(topic);
          if (channel_it == channels.end() ||
              frame.bag_log_time_ns < common_window.start_ns ||
              frame.bag_log_time_ns > common_window.end_ns) {
            return true;
          }
          if (current_bag_time &&
              frame.bag_log_time_ns < *current_bag_time) {
            throw std::runtime_error(
                "流式雷达帧未按bag_log_time递增");
          }
          if (current_bag_time &&
              frame.bag_log_time_ns > *current_bag_time &&
              !current_bag_packets.empty()) {
            throw std::runtime_error(
                "流式雷达上一bag_log_time包数未达到轻量预检");
          }
          current_bag_time = frame.bag_log_time_ns;
          const auto channel_order = static_cast<std::size_t>(
              std::distance(config.channels.data(), channel_it->second));
          const auto frame_order = next_frame_order[topic]++;
          for (std::size_t index = 0; index < frame.packets.size(); ++index) {
            auto payload = std::move(frame.packets[index]);
            const auto info =
                joint_sensor_player::inspect_lidar_packet(payload);
            const auto previous_time = previous_packet_time.find(topic);
            if (previous_time != previous_packet_time.end() &&
                info.sensor_time_us <= previous_time->second) {
              throw std::runtime_error(
                  "流式雷达包内时间违反轻量预检: " + topic);
            }
            const auto previous_sequence =
                previous_packet_sequence.find(topic);
            if (previous_sequence != previous_packet_sequence.end()) {
              const std::uint32_t delta =
                  info.sequence - previous_sequence->second;
              if (delta == 0 || delta >= 0x80000000U) {
                throw std::runtime_error(
                    "流式雷达序号违反轻量预检: " + topic);
              }
            }
            previous_packet_time[topic] = info.sensor_time_us;
            previous_packet_sequence[topic] = info.sequence;
            const auto target_time =
                checked_add(radar_anchor_us,
                            info.sensor_time_us - *source_packet_origin,
                            "流式雷达目标UTC溢出");
            joint_sensor_player::rewrite_lidar_packet(
                payload, target_time, info.sequence);
            const auto deadline =
                joint_sensor_player::packet_deadline_ns(
                    play_start_ns, common_window.start_ns,
                    frame.bag_log_time_ns, index,
                    channel_it->second->packet_interval_ns,
                    channel_it->second->time_offset_us);
            current_bag_packets.push_back(
                {channel_it->second, topic, std::move(payload),
                 frame.bag_log_time_ns, frame.bag_log_time_ns,
                 deadline, static_cast<std::uint32_t>(frame.packets.size()),
                 frame_order, static_cast<std::uint32_t>(index),
                 channel_order});
          }
          enqueue_completed_bag_time();
          return true;
        });
        if (!cancellation.stop_requested()) {
          enqueue_group(
              current_bag_packets,
              std::numeric_limits<std::uint64_t>::max());
        }
      } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(shared.mutex);
        shared.producer_error = error.what();
      } catch (...) {
        std::lock_guard<std::mutex> lock(shared.mutex);
        shared.producer_error = "流式雷达生产者发生未知异常";
      }
      {
        std::lock_guard<std::mutex> lock(shared.mutex);
        shared.producer_done = true;
      }
      shared.changed.notify_all();
    });

    struct Lane {
      std::mutex mutex;
      std::condition_variable cv;
      std::deque<ScheduledPacket> packets;
      bool closed{false};
    };
    std::vector<Lane> lanes(config.channels.size());
    std::mutex result_mutex;
    std::mutex sender_error_mutex;
    std::string sender_error;
    constexpr std::size_t lane_queue_limit = 256;
    const auto note_sender_error = [&](const std::string& message) {
      {
        std::lock_guard<std::mutex> lock(sender_error_mutex);
        if (sender_error.empty()) sender_error = message;
      }
      cancellation.request_stop();
      for (auto& lane : lanes) {
        std::lock_guard<std::mutex> lock(lane.mutex);
        lane.closed = true;
        lane.cv.notify_all();
      }
      shared.changed.notify_all();
    };
    std::exception_ptr dispatcher_error;
    std::vector<std::thread> senders;
    senders.reserve(lanes.size());
    for (std::size_t channel_order = 0; channel_order < lanes.size();
         ++channel_order) {
      senders.emplace_back([&, channel_order] {
        auto& lane = lanes[channel_order];
        try {
          while (true) {
            ScheduledPacket packet;
            {
              std::unique_lock<std::mutex> lock(lane.mutex);
              while (!lane.closed && !cancellation.stop_requested() &&
                     lane.packets.empty()) {
                lane.cv.wait_for(lock, std::chrono::milliseconds(1));
              }
              if (cancellation.stop_requested() || lane.packets.empty()) {
                break;
              }
              packet = std::move(lane.packets.front());
              lane.packets.pop_front();
            }
            lane.cv.notify_all();
            const auto wait =
                joint_sensor_player::cancellation_aware_sleep_until(
                    clock, packet.deadline_ns, &cancellation);
            if (wait == joint_sensor_player::WaitStatus::Cancelled) {
              lane.cv.notify_all();
              break;
            }
            if (wait == joint_sensor_player::WaitStatus::Error) {
              throw std::runtime_error("等待流式雷达绝对期限失败");
            }
            // 醒来时刻相对期限的差值就是这一包的迟到，sendmsg 还没发生。
            const auto before_send = clock.now_ns();
            if (before_send > packet.deadline_ns) {
              const auto lateness = before_send - packet.deadline_ns;
              {
                std::lock_guard<std::mutex> lock(result_mutex);
                result.max_lateness_ns =
                    std::max(result.max_lateness_ns, lateness);
              }
              if (lateness >
                  static_cast<std::uint64_t>(config.max_lateness_us) *
                      1000ULL) {
                throw std::runtime_error(
                    "流式雷达发送迟到超过门限: lateness_us=" +
                    std::to_string(lateness / 1000ULL) + " limit_us=" +
                    std::to_string(config.max_lateness_us));
              }
            }
            std::string error;
            if (!sink.send(*packet.channel, packet.payload,
                           packet.deadline_ns, error)) {
              throw std::runtime_error(
                  error.empty() ? "流式雷达UDP发送失败" : error);
            }
            const auto after_send = clock.now_ns();
            std::lock_guard<std::mutex> lock(result_mutex);
            ++result.packets_sent;
            if (packet.packet_index + 1 == packet.frame_packet_count) {
              ++result.frames_sent;
              const auto lateness =
                  after_send > packet.deadline_ns
                      ? after_send - packet.deadline_ns
                      : 0;
              if (frame_observer) {
                frame_observer(
                    {packet.channel->slot, packet.topic,
                     packet.source_time_ns, packet.deadline_ns, after_send,
                     packet.frame_packet_count, lateness});
              }
            }
          }
          lane.cv.notify_all();
        } catch (const std::exception& error) {
          note_sender_error(error.what());
        }
      });
    }

    std::string consumer_error;
    try {
      try {
        while (true) {
          ScheduledPacket packet;
          {
            std::unique_lock<std::mutex> lock(shared.mutex);
            const auto ready = [&] {
              return cancellation.stop_requested() ||
                     !shared.producer_error.empty() ||
                     (shared.producer_done && shared.packets.empty()) ||
                     (!shared.packets.empty() &&
                      (shared.producer_done ||
                       (shared.unread_deadline_lower_bound &&
                        packet_deadline_is_safe(
                            shared.packets.top().deadline_ns,
                            *shared.unread_deadline_lower_bound))));
            };
            while (!ready()) {
              shared.changed.wait_for(
                  lock, std::chrono::milliseconds(1));
            }
            if (cancellation.stop_requested()) break;
            if (!shared.producer_error.empty()) {
              throw std::runtime_error(shared.producer_error);
            }
            if (shared.producer_done && shared.packets.empty()) break;
            packet =
                std::move(const_cast<ScheduledPacket&>(shared.packets.top()));
            shared.packets.pop();
          }
          shared.changed.notify_all();
          auto& lane = lanes.at(packet.channel_order);
          {
            std::unique_lock<std::mutex> lane_lock(lane.mutex);
            while (!cancellation.stop_requested() && !lane.closed &&
                   lane.packets.size() >= lane_queue_limit) {
              lane.cv.wait_for(lane_lock, std::chrono::milliseconds(1));
            }
            if (cancellation.stop_requested() || lane.closed) break;
            lane.packets.push_back(std::move(packet));
          }
          lane.cv.notify_all();
        }
      } catch (...) {
        dispatcher_error = std::current_exception();
        cancellation.request_stop();
      }
      for (auto& lane : lanes) {
        std::lock_guard<std::mutex> lock(lane.mutex);
        lane.closed = true;
        lane.cv.notify_all();
      }
      for (auto& sender : senders) {
        if (sender.joinable()) sender.join();
      }
      if (dispatcher_error) std::rethrow_exception(dispatcher_error);
      std::lock_guard<std::mutex> lock(sender_error_mutex);
      if (!sender_error.empty()) {
        throw std::runtime_error(sender_error);
      }
    } catch (const std::exception& error) {
      consumer_error = error.what();
      cancellation.request_stop();
    }
    shared.changed.notify_all();
    if (producer.joinable()) producer.join();
    result.max_buffered_packets = shared.max_packets;
    if (!consumer_error.empty()) {
      result.error = consumer_error;
      result.stop_reason = result.error;
    } else if (cancellation.stop_requested()) {
      result.ok = true;
      result.cancelled = true;
      result.stop_reason = "cancelled";
    } else {
      result.ok = true;
      result.loops_completed = 1;
      result.stop_reason = "completed";
    }
  } catch (const std::exception& error) {
    cancellation.request_stop();
    result.error = error.what();
    result.stop_reason = result.error;
  }
  return result;
}

RadarEngine::RadarEngine(joint_sensor_player::PlayerConfig config,
                         SourceWindow common_window,
                         LifecycleManager& lifecycle,
                         RadarEngineDependencies dependencies)
    : config_(std::move(config)),
      common_window_(common_window),
      lifecycle_(lifecycle),
      dependencies_(std::move(dependencies)) {
  if (common_window_.start_ns > common_window_.end_ns) {
    throw std::invalid_argument("雷达共同窗口起点晚于终点");
  }
  if (dependencies_.clock == nullptr || dependencies_.sink == nullptr) {
    throw std::invalid_argument("RadarEngine需要Clock和DatagramSink");
  }
  if (!dependencies_.thread_factory) {
    dependencies_.thread_factory = [](std::function<void()> worker) {
      return std::thread(std::move(worker));
    };
  }
  config_.loop_count = 1;
}

RadarEngine::~RadarEngine() { stop(); }

void RadarEngine::prepare() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != RadarEngineState::Created) {
      throw std::logic_error("RadarEngine只能准备一次");
    }
  }
  try {
    if (dependencies_.prevalidated_lightweight_catalog) {
      lightweight_catalog_ =
          std::move(*dependencies_.prevalidated_lightweight_catalog);
      dependencies_.prevalidated_lightweight_catalog.reset();
      streaming_ = true;
    } else if (dependencies_.prevalidated_catalog) {
      catalog_ = std::move(*dependencies_.prevalidated_catalog);
      dependencies_.prevalidated_catalog.reset();
    } else if (dependencies_.scan) {
      catalog_ = dependencies_.scan(config_.data_dir, config_.channels);
    } else {
      const auto scan =
          dependencies_.lightweight_scan
              ? dependencies_.lightweight_scan
              : RadarLightweightScanFunction(
                    joint_sensor_player::scan_mcap_directory_lightweight);
      lightweight_catalog_ = scan(config_.data_dir, config_.channels);
      streaming_ = true;
    }
    if (streaming_) {
      if (lightweight_catalog_.required_reorder_frames >
          dependencies_.streaming_limits.frame_reorder_capacity) {
        throw std::runtime_error(
            "轻量雷达预检所需乱序帧缓冲超过上限: required=" +
            std::to_string(
                lightweight_catalog_.required_reorder_frames) +
            " limit=" +
            std::to_string(
                dependencies_.streaming_limits.frame_reorder_capacity));
      }
      const auto preflight = analyze_streaming_radar(
          config_, lightweight_catalog_, common_window_);
      if (preflight.required_packet_buffer >
          dependencies_.streaming_limits.packet_queue_capacity) {
        throw std::runtime_error(
            "轻量雷达预检所需packet heap超过上限: required=" +
            std::to_string(preflight.required_packet_buffer) +
            " limit=" +
            std::to_string(
                dependencies_.streaming_limits.packet_queue_capacity));
      }
      for (const auto& channel : config_.channels) {
        if (!channel.enabled) continue;
        const auto found =
            lightweight_catalog_.frames.find(channel.topic);
        if (found == lightweight_catalog_.frames.end()) {
          throw std::runtime_error(
              "轻量雷达预检缺少topic: " + channel.topic);
        }
        const bool selected = std::any_of(
            found->second.begin(), found->second.end(),
            [this](const auto& frame) {
              return frame.bag_log_time_ns >= common_window_.start_ns &&
                     frame.bag_log_time_ns <= common_window_.end_ns;
            });
        if (!selected) {
          throw std::runtime_error(
              "共同窗口内没有轻量雷达帧: " + channel.topic);
        }
      }
    } else {
      for (auto& [topic, frames] : catalog_.frames) {
        (void)topic;
        for (auto& frame : frames) {
          frame.source_timestamp_ns = frame.bag_log_time_ns;
        }
        std::stable_sort(
            frames.begin(), frames.end(),
            [](const auto& left, const auto& right) {
              return left.source_timestamp_ns < right.source_timestamp_ns;
            });
        frames.erase(
            std::remove_if(
                frames.begin(), frames.end(),
                [this](const joint_sensor_player::LidarFrame& frame) {
                  return frame.source_timestamp_ns <
                             common_window_.start_ns ||
                         frame.source_timestamp_ns >
                             common_window_.end_ns;
                }),
            frames.end());
      }
      (void)joint_sensor_player::build_playback_window(
          catalog_, config_.channels, false);
    }
    if (config_.ptc_enabled) {
      if (!dependencies_.ptc) {
        dependencies_.ptc =
            std::make_unique<ExistingPtcService>(config_);
      }
      dependencies_.ptc->start();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = RadarEngineState::Ready;
  } catch (const std::exception& error) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = RadarEngineState::Failed;
    }
    lifecycle_.fail("radar", error.what());
    stop_ptc();
    throw;
  }
}

void RadarEngine::arm(std::shared_ptr<StartGate> start_gate) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != RadarEngineState::Ready) {
    throw std::logic_error("RadarEngine必须READY后才能ARMED");
  }
  if (!start_gate) {
    throw std::invalid_argument("RadarEngine需要共享StartGate");
  }
  start_gate_ = std::move(start_gate);
  state_ = RadarEngineState::Armed;
}

void RadarEngine::run() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != RadarEngineState::Armed) {
      throw std::logic_error("RadarEngine必须ARMED后才能RUNNING");
    }
    state_ = RadarEngineState::Running;
  }
  try {
    worker_ = dependencies_.thread_factory(
        [this] { worker_main(); });
  } catch (const std::exception& error) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = RadarEngineState::Failed;
      result_.playback.error = error.what();
      result_.playback.stop_reason = error.what();
    }
    try {
      lifecycle_.fail("radar", error.what());
    } catch (...) {
    }
    start_gate_->cancel();
    stop_ptc();
    throw;
  }
}

// 参加 StartGate。锚点到手后进入流式发送。发送线程默认不绑核。
void RadarEngine::worker_main() noexcept {
  pin_current_thread(dependencies_.sender_cpu);
  joint_sensor_player::PlaybackResult playback;
  StartAnchors anchors;
  try {
    if (!start_gate_->arrive_and_wait(
            *lifecycle_.stop_token(), anchors)) {
      playback.ok = true;
      playback.cancelled = true;
      playback.stop_reason = "cancelled";
    } else if (streaming_) {
      const auto stream =
          dependencies_.stream
              ? dependencies_.stream
              : RadarFrameStreamFunction(
                    [this](
                        const joint_sensor_player::McapFrameObserver&
                            observer) {
                      return joint_sensor_player::stream_mcap_frames(
                          lightweight_catalog_, config_.channels,
                          {dependencies_.streaming_limits
                               .frame_reorder_capacity},
                          observer);
                    });
      playback = play_streaming_radar(
          config_, lightweight_catalog_, common_window_,
          *dependencies_.clock, *dependencies_.sink,
          *lifecycle_.stop_token(), anchors.utc_anchor_us,
          anchors.play_start_ns, stream,           dependencies_.frame_observer,
          dependencies_.streaming_limits, dependencies_.reader_cpu);
      result_.common_window = common_window_;
      result_.utc_anchor_us = anchors.utc_anchor_us;
    } else {
      const auto window = joint_sensor_player::build_playback_window(
          catalog_, config_.channels, false);
      const auto source_offset_ns =
          window.source_start_ns - common_window_.start_ns;
      const auto radar_play_start_ns = checked_add(
          anchors.play_start_ns, source_offset_ns, "雷达播放起点溢出");
      const auto radar_anchor_us = checked_add(
          anchors.utc_anchor_us, source_offset_ns / 1000ULL,
          "雷达UTC锚点溢出");
      rewriter_ =
          std::make_unique<joint_sensor_player::LidarPacketRewriter>(
              joint_sensor_player::LidarPacketRewriter::build_single_pass(
                  catalog_, config_.channels, window,
                  radar_anchor_us));
      playback = joint_sensor_player::play_catalog(
          config_, catalog_, *dependencies_.clock, *dependencies_.sink,
          nullptr, lifecycle_.stop_token().get(), rewriter_.get(),
          radar_play_start_ns, dependencies_.frame_observer);
      playback.play_start_ns = anchors.play_start_ns;
      result_.common_window = common_window_;
      result_.utc_anchor_us = anchors.utc_anchor_us;
    }
  } catch (const std::exception& error) {
    playback.error = error.what();
    playback.stop_reason = error.what();
  } catch (...) {
    playback.error = "雷达工作线程发生未知异常";
    playback.stop_reason = playback.error;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result_.playback = playback;
    state_ = playback.ok ? RadarEngineState::Stopped
                         : RadarEngineState::Failed;
  }
  if (!playback.ok) {
    try {
      lifecycle_.fail(
          "radar",
          playback.error.empty() ? "雷达工作线程异常退出" : playback.error);
    } catch (...) {
    }
  }
  stop_ptc();
}

void RadarEngine::stop_ptc() noexcept {
  if (dependencies_.ptc) dependencies_.ptc->stop();
}

void RadarEngine::stop() noexcept {
  lifecycle_.request_stop();
  if (start_gate_) start_gate_->cancel();
  if (worker_.joinable()) worker_.join();
  stop_ptc();
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != RadarEngineState::Failed) {
    state_ = RadarEngineState::Stopped;
  }
}

RadarEngineResult RadarEngine::result() {
  if (worker_.joinable()) worker_.join();
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != RadarEngineState::Stopped &&
      state_ != RadarEngineState::Failed) {
    throw std::logic_error("RadarEngine结果尚不可用");
  }
  return result_;
}

RadarEngineState RadarEngine::state() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

}  // namespace joint_sensor_orchestrator
