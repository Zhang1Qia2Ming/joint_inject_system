#pragma once

#include "joint_sensor_player/config.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace joint_sensor_player {

class DatagramSink {
 public:
  virtual ~DatagramSink() = default;
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
