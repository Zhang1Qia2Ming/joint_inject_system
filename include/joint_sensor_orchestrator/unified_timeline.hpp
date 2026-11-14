#pragma once

// 视频和雷达共用的源时间窗，以及一路视频在这个窗口里的输出槽。
// 源帧率低于 30 Hz 时，同一张源帧占连续几个输出槽。
// deadline_ns 是这张帧允许开始写卡的绝对单调时间。

#include <cstdint>
#include <vector>

namespace joint_sensor_orchestrator {

struct SourceWindow {
  std::uint64_t start_ns{};
  std::uint64_t end_ns{};
};

struct VideoSlot {
  std::uint64_t source_index{};
  // 本路发出去的第几帧，从 0 连续增加。
  std::uint64_t output_index{};
  // 30 Hz 网格上的序号。10 Hz 源帧会占相邻的三个格子。
  std::uint64_t grid_index{};
  std::uint64_t source_time_ns{};
  std::uint64_t slot_time_ns{};
  // 相对共同窗口起点的时间，用来和雷达对齐。
  std::uint64_t relative_ns{};
  // 写卡线程要睡到的绝对单调时间。
  std::uint64_t deadline_ns{};
};

// play_start_ns 为 0 时只用来数有哪些源帧会真正发出，不算绝对期限。
// filler 用这个结果决定读到哪一帧就停。
std::vector<VideoSlot> build_video_slots(
    const std::vector<std::uint64_t>& source_timestamps_ns,
    unsigned source_fps, unsigned output_fps, SourceWindow common_window,
    std::uint64_t play_start_ns);

}  // namespace joint_sensor_orchestrator
