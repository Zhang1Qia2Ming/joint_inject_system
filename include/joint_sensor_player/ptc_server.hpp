#pragma once

// 雷达标定 PTC 服务。域控连上来取 correction 文件。
// 它在发送开始前就得听着，但不参加 StartGate，也不碰视频。

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace joint_sensor_player {

std::vector<std::uint8_t> build_ptc_response(
    const std::vector<std::uint8_t>& request,
    const std::vector<std::uint8_t>& correction);

struct PtcEndpoint {
  std::string address;
  std::uint16_t port{};
  std::vector<std::uint8_t> correction;
};

class PtcCalibrationService {
 public:
  explicit PtcCalibrationService(std::vector<PtcEndpoint> endpoints);
  ~PtcCalibrationService();
  PtcCalibrationService(const PtcCalibrationService&) = delete;
  PtcCalibrationService& operator=(const PtcCalibrationService&) = delete;

  void start();
  std::vector<std::uint16_t> bound_ports() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace joint_sensor_player
