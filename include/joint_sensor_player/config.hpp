#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace joint_sensor_player {

enum class SequenceMode {
  Preserve,
  Continuous,
};

struct LidarChannelConfig {
  int slot{};
  std::string name;
  std::string topic;
  std::string source_ip;
  std::uint16_t source_port{10000};
  std::uint16_t destination_port{};
  std::uint32_t packet_size{};
  std::uint32_t packets_per_frame{};
  std::uint64_t packet_interval_ns{};
  std::int64_t time_offset_us{};
  bool enabled{true};
  std::filesystem::path correction_file;
};

struct PlayerConfig {
  std::filesystem::path data_dir{"/data/joint_sensor_player/data"};
  std::string interface{"enp3s0f0"};
  std::string destination_ip{"192.168.1.4"};
  bool allow_partial_start{false};
  std::uint32_t start_delay_ms{2000};
  std::uint32_t max_lateness_us{20000};
  std::uint64_t loop_count{1};
  SequenceMode sequence_mode{SequenceMode::Continuous};
  bool require_synchronized_clock{true};
  bool ptc_enabled{false};
  std::uint16_t ptc_port{9347};
  std::filesystem::path report_dir{
      "/home/ubuntu/zhang/joint_sensor_player/reports"};
  std::vector<LidarChannelConfig> channels;
};

std::array<LidarChannelConfig, 8> default_lidar_channels();
PlayerConfig load_config(const std::filesystem::path& path);
void validate_config(const PlayerConfig& config);

}  // namespace joint_sensor_player
