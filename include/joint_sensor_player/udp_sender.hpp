#pragma once

// 雷达 UDP 出口。每路按配置里的源地址和目的端口发出一个数据报。
// 联合播放把同一个 sink 交给 RadarEngine，视频不经过这里。

#include "joint_sensor_player/config.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace joint_sensor_player {

class DatagramSink {
 public:
  virtual ~DatagramSink() = default;
  // 发出一个包。调用前已经睡到期限，这里不再等待。
  virtual bool send(const LidarChannelConfig& channel,
                    const std::vector<std::uint8_t>& payload,
                    std::uint64_t deadline_ns, std::string& error) = 0;
};

class UdpDatagramSink final : public DatagramSink {
 public:
  explicit UdpDatagramSink(const PlayerConfig& config);
  ~UdpDatagramSink() override;
  UdpDatagramSink(const UdpDatagramSink&) = delete;
  UdpDatagramSink& operator=(const UdpDatagramSink&) = delete;

  bool send(const LidarChannelConfig& channel,
            const std::vector<std::uint8_t>& payload,
            std::uint64_t deadline_ns, std::string& error) override;

 private:
  struct Target;
  std::map<int, Target> targets_;
};

}  // namespace joint_sensor_player
