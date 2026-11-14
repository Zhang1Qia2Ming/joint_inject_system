// 把一路视频的源时间戳铺到 30 Hz 输出槽上，并加上 play_start 得到绝对期限。
// 10 Hz 源帧连续占三个槽。窗口尾部没有输出槽的源帧不会出现在结果里。

#include "joint_sensor_orchestrator/unified_timeline.hpp"

#include <limits>
#include <stdexcept>

namespace joint_sensor_orchestrator {
namespace {

constexpr std::uint64_t kNanosecondsPerSecond = 1'000'000'000ULL;
constexpr std::uint64_t kOutputFps = 30;

std::uint64_t grid_relative_ns(std::uint64_t grid_index) {
  const std::uint64_t seconds = grid_index / kOutputFps;
  if (seconds >
      std::numeric_limits<std::uint64_t>::max() / kNanosecondsPerSecond) {
    throw std::overflow_error("视频网格时间溢出");
  }
  const std::uint64_t whole = seconds * kNanosecondsPerSecond;
  const std::uint64_t fraction =
      ((grid_index % kOutputFps) * kNanosecondsPerSecond) / kOutputFps;
  if (whole > std::numeric_limits<std::uint64_t>::max() - fraction) {
    throw std::overflow_error("视频网格时间溢出");
  }
  return whole + fraction;
}

std::uint64_t nearest_grid_index(std::uint64_t relative_ns) {
  const std::uint64_t seconds = relative_ns / kNanosecondsPerSecond;
  if (seconds >
      std::numeric_limits<std::uint64_t>::max() / kOutputFps) {
    throw std::overflow_error("视频网格序号溢出");
  }
  const std::uint64_t remainder = relative_ns % kNanosecondsPerSecond;
  const std::uint64_t rounded =
      (remainder * kOutputFps + kNanosecondsPerSecond / 2) /
      kNanosecondsPerSecond;
  const std::uint64_t base = seconds * kOutputFps;
  if (base > std::numeric_limits<std::uint64_t>::max() - rounded) {
    throw std::overflow_error("视频网格序号溢出");
  }
  return base + rounded;
}

std::uint64_t checked_add(std::uint64_t left, std::uint64_t right,
                          const char* message) {
  if (left > std::numeric_limits<std::uint64_t>::max() - right) {
    throw std::overflow_error(message);
  }
  return left + right;
}

}  // namespace

std::vector<VideoSlot> build_video_slots(
    const std::vector<std::uint64_t>& source_timestamps_ns,
    unsigned source_fps, unsigned output_fps, SourceWindow common_window,
    std::uint64_t play_start_ns) {
  if (common_window.start_ns > common_window.end_ns) {
    throw std::invalid_argument("共同时间窗口起点晚于终点");
  }
  if (output_fps != 30 || (source_fps != 10 && source_fps != 30)) {
    throw std::invalid_argument("仅支持 10->30 和 30->30 视频映射");
  }
  for (std::size_t index = 1; index < source_timestamps_ns.size(); ++index) {
    if (source_timestamps_ns[index] <= source_timestamps_ns[index - 1]) {
      throw std::invalid_argument("视频源时间戳必须严格递增");
    }
  }

  const unsigned repeats = output_fps / source_fps;
  std::vector<VideoSlot> slots;
  if (source_timestamps_ns.size() >
      std::numeric_limits<std::size_t>::max() / repeats) {
    throw std::overflow_error("视频输出槽数量溢出");
  }
  slots.reserve(source_timestamps_ns.size() * repeats);

  bool have_previous_grid{};
  std::uint64_t previous_grid{};
  for (std::size_t source_index = 0;
       source_index < source_timestamps_ns.size(); ++source_index) {
    const std::uint64_t source_time = source_timestamps_ns[source_index];
    if (source_time < common_window.start_ns ||
        source_time > common_window.end_ns) {
      continue;
    }
    std::uint64_t first_grid =
        nearest_grid_index(source_time - common_window.start_ns);
    if (have_previous_grid && first_grid <= previous_grid) {
      if (previous_grid == std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error("视频网格序号溢出");
      }
      first_grid = previous_grid + 1;
    }
    if (first_grid >
        std::numeric_limits<std::uint64_t>::max() - (repeats - 1)) {
      throw std::overflow_error("视频网格序号溢出");
    }
    const std::uint64_t last_grid = first_grid + (repeats - 1);
    const std::uint64_t last_relative = grid_relative_ns(last_grid);
    const std::uint64_t last_slot_time =
        checked_add(common_window.start_ns, last_relative,
                    "视频输出槽时间溢出");
    if (last_slot_time > common_window.end_ns) {
      continue;
    }
    for (unsigned repeat = 0; repeat < repeats; ++repeat) {
      const std::uint64_t grid_index = first_grid + repeat;
      const std::uint64_t relative = grid_relative_ns(grid_index);
      const std::uint64_t slot_time =
          checked_add(common_window.start_ns, relative,
                      "视频输出槽时间溢出");
      const std::uint64_t deadline =
          checked_add(play_start_ns, relative, "视频发送期限溢出");
      previous_grid = grid_index;
      have_previous_grid = true;
      slots.push_back(VideoSlot{
          static_cast<std::uint64_t>(source_index),
          static_cast<std::uint64_t>(slots.size()),
          grid_index,
          source_time,
          slot_time,
          relative,
          deadline});
    }
  }
  return slots;
}

}  // namespace joint_sensor_orchestrator
