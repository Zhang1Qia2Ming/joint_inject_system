// 读取八路雷达 YAML，并检查地址、端口和标定文件是否能开播。

#include "joint_sensor_player/config.hpp"

#include <arpa/inet.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <set>
#include <stdexcept>

namespace joint_sensor_player {
namespace {

LidarChannelConfig main_lidar(int slot, std::string name, std::string topic,
                              std::uint16_t port) {
  return {slot,
          std::move(name),
          std::move(topic),
          "192.168.1." + std::to_string(slot),
          10000,
          port,
          1118,
          626,
          83200,
          0,
          true,
          {}};
}

LidarChannelConfig short_lidar(int slot, std::string name, std::string topic,
                               std::uint16_t port) {
  return {slot,
          std::move(name),
          std::move(topic),
          "192.168.1." + std::to_string(slot),
          10000,
          port,
          554,
          512,
          193300,
          0,
          true,
          {}};
}

template <typename T>
void set_if_present(const YAML::Node& node, const char* key, T& value) {
  if (node[key]) value = node[key].as<T>();
}

void set_if_present(const YAML::Node& node, const char* key,
                    std::filesystem::path& value) {
  if (node[key]) value = node[key].as<std::string>();
}

}  // namespace

std::array<LidarChannelConfig, 8> default_lidar_channels() {
  return {
      main_lidar(31, "front_main", "/sensor/lidar_front/orig/udp_packets", 2368),
      main_lidar(32, "left_main", "/sensor/lidar_left/orig/udp_packets", 2369),
      main_lidar(33, "right_main", "/sensor/lidar_right/orig/udp_packets", 2370),
      main_lidar(34, "rear_main", "/sensor/lidar_rear/orig/udp_packets", 2371),
      short_lidar(35, "front_short",
                  "/sensor/short_lidar_front/orig/udp_packets", 2335),
      short_lidar(36, "left_short",
                  "/sensor/short_lidar_left/orig/udp_packets", 2336),
      short_lidar(37, "right_short",
                  "/sensor/short_lidar_right/orig/udp_packets", 2337),
      short_lidar(38, "rear_short",
                  "/sensor/short_lidar_rear/orig/udp_packets", 2338),
  };
}

void validate_config(const PlayerConfig& config) {
  if (config.channels.empty()) throw std::runtime_error("未配置雷达通道");
  if (config.max_lateness_us == 0) {
    throw std::runtime_error("max_lateness_us必须大于0");
  }
  if (config.interface.empty() || config.destination_ip.empty()) {
    throw std::runtime_error("网络接口和目标IP不能为空");
  }
  if (config.ptc_enabled && config.ptc_port == 0) {
    throw std::runtime_error("PTC端口必须大于0");
  }
  in_addr parsed_address{};
  if (::inet_pton(AF_INET, config.destination_ip.c_str(), &parsed_address) !=
      1) {
    throw std::runtime_error("目标IP格式非法");
  }
  std::set<int> slots;
  std::set<std::string> topics;
  std::set<std::string> source_ips;
  std::set<std::uint16_t> destination_ports;
  for (const auto& channel : config.channels) {
    if (!channel.enabled) continue;
    if (!slots.insert(channel.slot).second ||
        !topics.insert(channel.topic).second ||
        !source_ips.insert(channel.source_ip).second ||
        !destination_ports.insert(channel.destination_port).second) {
      throw std::runtime_error("雷达slot、topic、源IP和目标端口必须唯一");
    }
    if (channel.packet_size == 0 || channel.packets_per_frame == 0 ||
        channel.packet_interval_ns == 0) {
      throw std::runtime_error("雷达包参数必须大于0");
    }
    if (channel.source_port == 0 || channel.destination_port == 0 ||
        ::inet_pton(AF_INET, channel.source_ip.c_str(), &parsed_address) != 1) {
      throw std::runtime_error("雷达源IP或端口非法");
    }
    if (config.ptc_enabled &&
        (channel.correction_file.empty() ||
         !std::filesystem::is_regular_file(channel.correction_file))) {
      throw std::runtime_error("PTC缺少有效correction_file: slot=" +
                               std::to_string(channel.slot));
    }
    if (channel.time_offset_us < -1000000 ||
        channel.time_offset_us > 1000000) {
      throw std::runtime_error("time_offset_us必须在正负1秒内");
    }
  }
}

PlayerConfig load_config(const std::filesystem::path& path) {
  PlayerConfig config;
  const auto defaults = default_lidar_channels();
  config.channels.assign(defaults.begin(), defaults.end());

  const YAML::Node root = YAML::LoadFile(path.string());
  set_if_present(root, "data_dir", config.data_dir);
  if (const auto network = root["network"]) {
    set_if_present(network, "interface", config.interface);
    set_if_present(network, "destination_ip", config.destination_ip);
  }
  if (const auto playback = root["playback"]) {
    set_if_present(playback, "allow_partial_start", config.allow_partial_start);
    set_if_present(playback, "start_delay_ms", config.start_delay_ms);
    set_if_present(playback, "max_lateness_us", config.max_lateness_us);
    set_if_present(playback, "loop_count", config.loop_count);
    if (playback["sequence_mode"]) {
      const auto mode = playback["sequence_mode"].as<std::string>();
      if (mode == "continuous") {
        config.sequence_mode = SequenceMode::Continuous;
      } else if (mode == "preserve") {
        config.sequence_mode = SequenceMode::Preserve;
      } else {
        throw std::runtime_error(
            "sequence_mode必须是continuous或preserve");
      }
    }
  }
  if (const auto clock = root["clock"]) {
    set_if_present(clock, "require_synchronized",
                   config.require_synchronized_clock);
  }
  if (const auto ptc = root["ptc"]) {
    set_if_present(ptc, "enabled", config.ptc_enabled);
    set_if_present(ptc, "port", config.ptc_port);
  }
  if (const auto report = root["report"]) {
    set_if_present(report, "output_dir", config.report_dir);
  }
  if (const auto lidars = root["lidars"]) {
    if (!lidars.IsSequence()) throw std::runtime_error("lidars必须是列表");
    for (const auto& override_node : lidars) {
      const int slot = override_node["slot"].as<int>();
      const auto found = std::find_if(
          config.channels.begin(), config.channels.end(),
          [slot](const auto& channel) { return channel.slot == slot; });
      if (found == config.channels.end()) {
        throw std::runtime_error("未知雷达slot: " + std::to_string(slot));
      }
      set_if_present(override_node, "name", found->name);
      set_if_present(override_node, "topic", found->topic);
      set_if_present(override_node, "source_ip", found->source_ip);
      set_if_present(override_node, "source_port", found->source_port);
      set_if_present(override_node, "destination_port", found->destination_port);
      set_if_present(override_node, "packet_interval_ns",
                     found->packet_interval_ns);
      set_if_present(override_node, "time_offset_us", found->time_offset_us);
      set_if_present(override_node, "enabled", found->enabled);
      set_if_present(override_node, "correction_file",
                     found->correction_file);
    }
  }
  validate_config(config);
  return config;
}

}  // namespace joint_sensor_player
