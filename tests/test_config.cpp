#include "joint_sensor_player/config.hpp"
#include "test_framework.hpp"

#include <filesystem>
#include <fstream>

using joint_sensor_player::default_lidar_channels;
using joint_sensor_player::load_config;
using joint_sensor_player::SequenceMode;

TEST_CASE(default_profiles_match_domain_contract) {
  const auto channels = default_lidar_channels();
  CHECK(channels.size() == 8);
  CHECK(channels[0].slot == 31);
  CHECK(channels[0].source_ip == "192.168.1.31");
  CHECK(channels[0].destination_port == 2368);
  CHECK(channels[0].packet_interval_ns == 83200);
  CHECK(channels[4].slot == 35);
  CHECK(channels[4].destination_port == 2335);
  CHECK(channels[4].packet_interval_ns == 193300);
  CHECK(channels[7].source_ip == "192.168.1.38");
  CHECK(channels[7].destination_port == 2338);
}

TEST_CASE(config_loads_defaults_and_overrides) {
  const auto path = std::filesystem::temp_directory_path() / "joint_player_config.yaml";
  {
    std::ofstream out(path);
    out << "data_dir: /tmp/lidar-data\n"
           "network:\n"
           "  interface: enp3s0f0\n"
           "  destination_ip: 192.168.1.4\n"
           "playback:\n"
           "  allow_partial_start: false\n"
           "  start_delay_ms: 1500\n"
           "report:\n"
           "  output_dir: /tmp/reports\n";
  }
  const auto config = load_config(path);
  std::filesystem::remove(path);
  CHECK(config.data_dir == "/tmp/lidar-data");
  CHECK(config.interface == "enp3s0f0");
  CHECK(config.destination_ip == "192.168.1.4");
  CHECK(!config.allow_partial_start);
  CHECK(config.start_delay_ms == 1500);
  CHECK(config.channels.size() == 8);
}

TEST_CASE(config_rejects_duplicate_destination_ports) {
  const auto path = std::filesystem::temp_directory_path() / "joint_player_bad.yaml";
  {
    std::ofstream out(path);
    out << "lidars:\n"
           "  - slot: 31\n"
           "    destination_port: 2369\n";
  }
  CHECK_THROWS(load_config(path));
  std::filesystem::remove(path);
}

TEST_CASE(ptc_config_requires_and_loads_per_lidar_correction) {
  const auto root =
      std::filesystem::temp_directory_path() / "joint_player_ptc_config";
  std::filesystem::create_directories(root);
  const auto correction = root / "front.dat";
  {
    std::ofstream out(correction, std::ios::binary);
    out << "calibration";
  }
  const auto path = root / "config.yaml";
  {
    std::ofstream out(path);
    out << "ptc:\n"
           "  enabled: true\n"
           "  port: 9347\n"
           "lidars:\n"
           "  - slot: 31\n"
           "    correction_file: "
        << correction.string() << "\n";
  }
  CHECK_THROWS(load_config(path));

  {
    std::ofstream out(path);
    out << "ptc:\n"
           "  enabled: true\n"
           "  port: 9347\n"
           "lidars:\n";
    for (int slot = 31; slot <= 38; ++slot) {
      out << "  - slot: " << slot << "\n"
          << "    correction_file: " << correction.string() << "\n";
    }
  }
  const auto config = load_config(path);
  CHECK(config.ptc_enabled);
  CHECK(config.ptc_port == 9347);
  CHECK(config.channels[0].correction_file == correction);
  std::filesystem::remove_all(root);
}

TEST_CASE(loop_config_defaults_to_one_continuous_cycle) {
  const auto path =
      std::filesystem::temp_directory_path() / "joint_player_loop_default.yaml";
  {
    std::ofstream out(path);
    out << "data_dir: /tmp/lidar-data\n";
  }
  const auto config = load_config(path);
  std::filesystem::remove(path);
  CHECK(config.loop_count == 1);
  CHECK(config.sequence_mode == SequenceMode::Continuous);
}

TEST_CASE(loop_config_accepts_infinite_preserve_and_rejects_unknown_mode) {
  const auto path =
      std::filesystem::temp_directory_path() / "joint_player_loop_mode.yaml";
  {
    std::ofstream out(path);
    out << "playback:\n"
           "  loop_count: 0\n"
           "  sequence_mode: preserve\n";
  }
  const auto config = load_config(path);
  CHECK(config.loop_count == 0);
  CHECK(config.sequence_mode == SequenceMode::Preserve);
  {
    std::ofstream out(path);
    out << "playback:\n"
           "  sequence_mode: rewrite_everything\n";
  }
  CHECK_THROWS(load_config(path));
  std::filesystem::remove(path);
}
