#pragma once

// 给发送线程选核。跳过物理核 0，每个角色只占一个物理核上的一个逻辑 CPU。
// 视频写卡、生产和预读各占一核。雷达发送和读取保持不绑核。
// 四路环视只在还剩至少四个核时，占用编号最高的那四个。

#include <array>
#include <vector>

namespace joint_sensor_orchestrator {

struct SendAffinity {
  bool enabled{false};
  std::array<int, 3> video_consumers{-1, -1, -1};
  std::array<int, 3> video_producers{-1, -1, -1};
  int filler{-1};
  int radar_sender{-1};
  int radar_reader{-1};
// 四路环视模拟。只在视频角色分完后还剩至少四个核时才赋值，取编号最高的核。
  std::array<int, 4> shadow{-1, -1, -1, -1};
};

// 按在线物理核排出上面的分配。核不够时视频仍可启用，环视保持 -1。
// 雷达两个角色始终是 -1：发送线程绑到网卡队列的核上会让首秒雷达迟到变大。
SendAffinity plan_send_affinity(const std::vector<std::vector<int>>& cores);

std::vector<std::vector<int>> online_physical_cores();

// cpu < 0 时什么也不做。失败只返回 false，调用方继续跑。
bool pin_current_thread(int cpu);

}  // namespace joint_sensor_orchestrator
