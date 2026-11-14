// 不访问硬件。按 30 Hz 网格检查到来时刻和帧大小，用来跑没有注入卡的测试。

#include "joint_sensor_orchestrator/video_sink.hpp"

#include <algorithm>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace joint_sensor_orchestrator {
namespace {

bool nearest_grid_index(std::uint64_t origin_ns, std::uint64_t source_ns,
                        unsigned output_fps, std::uint64_t& result) {
  if (source_ns < origin_ns || output_fps == 0) {
    return false;
  }
  const std::uint64_t relative = source_ns - origin_ns;
  const std::uint64_t seconds = relative / 1'000'000'000ULL;
  if (seconds >
      std::numeric_limits<std::uint64_t>::max() / output_fps) {
    return false;
  }
  const std::uint64_t remainder = relative % 1'000'000'000ULL;
  if (remainder >
      (std::numeric_limits<std::uint64_t>::max() -
       500'000'000ULL) /
          output_fps) {
    return false;
  }
  const std::uint64_t rounded =
      (remainder * output_fps + 500'000'000ULL) / 1'000'000'000ULL;
  const std::uint64_t base = seconds * output_fps;
  if (base > std::numeric_limits<std::uint64_t>::max() - rounded) {
    return false;
  }
  result = base + rounded;
  return true;
}

}  // namespace

struct MockVideoSink::State {
  struct LaneState {
    bool seen{};
    unsigned source_repeats{};
    unsigned source_fps{};
    std::uint64_t source_first_grid{};
    VideoSlot previous;
  };

  explicit State(MockVideoSinkOptions value) : options(std::move(value)) {}

  VideoSinkResult fail(std::string reason) {
    if (health.ok) {
      health.ok = false;
      health.error = reason;
    }
    return {false, std::move(reason)};
  }

  mutable std::mutex mutex;
  MockVideoSinkOptions options;
  std::unordered_map<std::string, unsigned> lane_rates;
  std::unordered_map<std::string, LaneState> lanes;
  VideoSinkHealth health;
  std::optional<std::uint64_t> inferred_play_start_ns;
  bool prepared{};
  bool stopped{};
};

MockVideoSink::MockVideoSink(MockVideoSinkOptions options)
    : state_(std::make_unique<State>(std::move(options))) {}

MockVideoSink::~MockVideoSink() = default;

VideoSinkResult MockVideoSink::prepare() {
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (state_->prepared || state_->stopped) {
    return state_->fail("Mock VideoSink 生命周期状态无效");
  }
  if (state_->options.frame_bytes == 0 || state_->options.lanes.empty() ||
      state_->options.output_fps != 30) {
    return state_->fail("Mock VideoSink 配置无效");
  }
  for (const auto& lane : state_->options.lanes) {
    if (lane.name.empty() || (lane.source_fps != 10 && lane.source_fps != 30) ||
        state_->options.output_fps % lane.source_fps != 0 ||
        !state_->lane_rates.emplace(lane.name, lane.source_fps).second) {
      return state_->fail("Mock VideoSink lane 为空或重复");
    }
  }
  for (const auto& failure : state_->options.failures) {
    if (!state_->lane_rates.contains(failure.lane)) {
      return state_->fail("Mock 故障注入引用未知 lane");
    }
  }
  state_->prepared = true;
  return {true, {}};
}

VideoSinkResult MockVideoSink::write_frame(
    const std::string& lane, std::span<const std::byte> frame,
    const VideoSlot& slot, std::uint64_t actual_write_ns) {
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (!state_->prepared || state_->stopped) {
    return state_->fail("Mock VideoSink 尚未准备或已经停止");
  }
  if (!state_->health.ok) {
    return {false, state_->health.error};
  }

  const auto rate = state_->lane_rates.find(lane);
  if (rate == state_->lane_rates.end()) {
    return state_->fail("未知视频 lane: " + lane);
  }
  if (frame.size() != state_->options.frame_bytes) {
    return state_->fail("视频帧尺寸错误");
  }
  const std::uint64_t grid_seconds =
      slot.grid_index / state_->options.output_fps;
  if (grid_seconds >
      std::numeric_limits<std::uint64_t>::max() / 1'000'000'000ULL) {
    return state_->fail("输出槽网格溢出");
  }
  const std::uint64_t expected_relative =
      grid_seconds * 1'000'000'000ULL +
      ((slot.grid_index % state_->options.output_fps) *
       1'000'000'000ULL) /
          state_->options.output_fps;
  if (state_->options.play_start_ns == 0 &&
      !state_->inferred_play_start_ns) {
    if (slot.deadline_ns < expected_relative) {
      return state_->fail("视频槽期限早于相对时间");
    }
    state_->inferred_play_start_ns =
        slot.deadline_ns - expected_relative;
  }
  const auto play_start_ns =
      state_->options.play_start_ns != 0
          ? state_->options.play_start_ns
          : *state_->inferred_play_start_ns;
  if (state_->options.grid_origin_ns >
          std::numeric_limits<std::uint64_t>::max() - expected_relative ||
      play_start_ns >
          std::numeric_limits<std::uint64_t>::max() - expected_relative ||
      slot.relative_ns != expected_relative ||
      slot.slot_time_ns !=
          state_->options.grid_origin_ns + expected_relative ||
      slot.deadline_ns != play_start_ns + expected_relative) {
    return state_->fail("视频槽不在统一 30Hz 整数网格");
  }

  std::uint64_t nearest_source_grid{};
  if (!nearest_grid_index(
          state_->options.grid_origin_ns, slot.source_time_ns,
          state_->options.output_fps, nearest_source_grid)) {
    return state_->fail("源帧时间无法映射到全局视频网格");
  }

  auto& lane_state = state_->lanes[lane];
  if (!lane_state.seen) {
    if (slot.output_index != 0 || slot.grid_index != nearest_source_grid) {
      return state_->fail("首源帧未映射到最近全局网格槽");
    }
    lane_state.source_fps = rate->second;
    lane_state.source_repeats = 1;
    lane_state.source_first_grid = slot.grid_index;
  } else {
    const auto& previous = lane_state.previous;
    if (slot.output_index != previous.output_index + 1 ||
        slot.grid_index <= previous.grid_index) {
      return state_->fail("输出序号必须连续且全局网格必须严格递增");
    }
    if (lane_state.source_fps == 30) {
      if (slot.source_index != previous.source_index + 1 ||
          slot.source_time_ns <= previous.source_time_ns ||
          slot.grid_index !=
              std::max(nearest_source_grid, previous.grid_index + 1)) {
        return state_->fail("30Hz lane 必须一槽一源帧");
      }
      lane_state.source_repeats = 1;
      lane_state.source_first_grid = slot.grid_index;
    } else if (slot.source_index == previous.source_index) {
      if (lane_state.source_repeats >= 3 ||
          slot.source_time_ns != previous.source_time_ns ||
          slot.grid_index !=
              lane_state.source_first_grid + lane_state.source_repeats) {
        return state_->fail("10Hz lane 重复槽模式错误");
      }
      ++lane_state.source_repeats;
    } else if (slot.source_index == previous.source_index + 1) {
      if (lane_state.source_repeats != 3 ||
          slot.source_time_ns <= previous.source_time_ns ||
          slot.grid_index !=
              std::max(nearest_source_grid, previous.grid_index + 1)) {
        return state_->fail("10Hz lane 每个源帧必须连续占三个槽");
      }
      lane_state.source_repeats = 1;
      lane_state.source_first_grid = slot.grid_index;
    } else {
      return state_->fail("源帧序号不连续");
    }
  }

  for (const auto& failure : state_->options.failures) {
    if (failure.lane != lane || failure.output_index != slot.output_index) {
      continue;
    }
    switch (failure.kind) {
      case MockFailureKind::WriteFailure:
        return state_->fail("Mock 注入写入失败");
      case MockFailureKind::Timeout:
        return state_->fail("Mock 注入写入超时");
      case MockFailureKind::ShortWrite:
        return state_->fail("Mock 注入短写");
    }
  }

  const std::uint64_t lateness =
      actual_write_ns > slot.deadline_ns ? actual_write_ns - slot.deadline_ns
                                         : 0;
  if (lateness > state_->options.max_lateness_ns) {
    return state_->fail("视频写入迟到超过门限");
  }

  lane_state.seen = true;
  lane_state.previous = slot;
  ++state_->health.frames_written;
  state_->health.bytes_written += frame.size();
  if (lateness > state_->health.max_lateness_ns) {
    state_->health.max_lateness_ns = lateness;
  }
  return {true, {}};
}

VideoSinkHealth MockVideoSink::health() const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->health;
}

void MockVideoSink::stop() noexcept {
  std::lock_guard<std::mutex> lock(state_->mutex);
  state_->stopped = true;
}

}  // namespace joint_sensor_orchestrator
