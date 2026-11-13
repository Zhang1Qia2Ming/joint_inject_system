#include "joint_sensor_orchestrator/orchestrator_config.hpp"
#include "test_framework.hpp"

#include <filesystem>
#include <fstream>
#include <string>

namespace {

class TemporaryConfig {
 public:
  explicit TemporaryConfig(const std::string& text)
      : path_(std::filesystem::temp_directory_path() /
              ("joint_sensor_orchestrator_" +
               std::to_string(next_id_++) + ".yaml")) {
    std::ofstream out(path_);
    out << text;
  }

  ~TemporaryConfig() { std::filesystem::remove(path_); }

  const std::filesystem::path& path() const { return path_; }

 private:
  inline static unsigned next_id_{};
  std::filesystem::path path_;
};

joint_sensor_orchestrator::OrchestratorConfig load_config_text(
    const std::string& text) {
  TemporaryConfig config(text);
  return joint_sensor_orchestrator::load_orchestrator_config(config.path());
}

const char* kValidLanes =
    "video:\n"
    "  mcap: /tmp/video.mcap\n"
    "  cache_dir: /tmp/video-cache\n"
    "  lanes:\n"
    "    - name: tx1\n"
    "      topic: /camera/tx1/h265\n"
    "      h2c: 0\n"
    "      source_fps: 10\n"
    "      output_fps: 30\n"
    "    - name: tx2\n"
    "      topic: /camera/tx2/h265\n"
    "      h2c: 1\n"
    "      source_fps: 30\n"
    "      output_fps: 30\n"
    "    - name: tx4\n"
    "      topic: /camera/tx4/h265\n"
    "      h2c: 2\n"
    "      source_fps: 10\n"
    "      output_fps: 30\n";

}  // namespace

using joint_sensor_orchestrator::VideoBackend;
using joint_sensor_orchestrator::load_orchestrator_config;

TEST_CASE(orchestrator_config_is_mock_one_shot_and_bounded) {
  TemporaryConfig file(kValidLanes);
  const auto config = load_orchestrator_config(file.path());
  CHECK(config.video_backend == VideoBackend::Mock);
  CHECK(config.strict);
  CHECK(config.loop_count == 1);
  CHECK(!config.allow_video_only_smoke);
  CHECK(config.play_duration_ms == 0);
  CHECK(config.video_prefetch_ms == 0);
  CHECK(!config.wait_for_enter);
  CHECK(config.video_queue_frames == 5);
  CHECK(config.video_max_lateness_us == 5000);
  CHECK(config.lidar_max_lateness_us == 20000);
  CHECK(config.lanes[0].name == "tx1");
  CHECK(config.lanes[2].source_fps == 10);
}

TEST_CASE(orchestrator_config_loads_xdma_smoke_and_duration) {
  const auto config = load_config_text(
      "mode:\n"
      "  video_backend: xdma\n"
      "  strict: true\n"
      "  allow_video_only_smoke: true\n"
      "playback:\n"
      "  loop_count: 1\n"
      "  play_duration_ms: 10000\n" +
      std::string(kValidLanes));
  CHECK(config.video_backend == VideoBackend::Xdma);
  CHECK(config.allow_video_only_smoke);
  CHECK(config.play_duration_ms == 10000);
  CHECK(!config.wait_for_enter);
}

TEST_CASE(orchestrator_config_loads_video_prefetch_ms) {
  const auto config = load_config_text(
      "playback:\n"
      "  video_prefetch_ms: 5000\n" +
      std::string(kValidLanes));
  CHECK(config.video_prefetch_ms == 5000);
}

TEST_CASE(orchestrator_config_loads_40s_ssd_prefetch_yaml) {
  const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() /
                    "config/orchestrator_xdma_joint_40s.yaml";
  const auto config = load_orchestrator_config(path);
  CHECK(config.play_duration_ms == 0);
  CHECK(config.video_prefetch_ms == 6500);
  CHECK(config.video_max_lateness_us == 5000);
  CHECK(config.lidar_max_lateness_us == 100000);
  CHECK(config.video_backend == VideoBackend::Xdma);
  CHECK(config.lanes[0].name == "tx1");
  CHECK(config.lanes[1].name == "tx2");
  CHECK(config.lanes[2].name == "tx4");
}

TEST_CASE(orchestrator_config_loads_wait_for_enter) {
  const auto config = load_config_text(
      "playback:\n"
      "  wait_for_enter: true\n" +
      std::string(kValidLanes));
  CHECK(config.wait_for_enter);
}

TEST_CASE(orchestrator_rejects_video_only_smoke_without_xdma) {
  CHECK_THROWS(load_config_text(
      "mode:\n"
      "  video_backend: mock\n"
      "  allow_video_only_smoke: true\n" +
      std::string(kValidLanes)));
}

TEST_CASE(orchestrator_config_loads_explicit_overrides) {
  const auto config = load_config_text(
      "mode:\n"
      "  video_backend: xdma\n"
      "  strict: true\n"
      "playback:\n"
      "  loop_count: 1\n"
      "timing:\n"
      "  video_max_lateness_us: 6000\n"
      "  lidar_max_lateness_us: 21000\n"
      "video:\n"
      "  mcap: /tmp/other.mcap\n"
      "  cache_dir: /tmp/other-cache\n"
      "  queue_frames: 3\n"
      "  lanes:\n"
      "    - {name: tx1, topic: /tx1, h2c: 0, source_fps: 10, output_fps: 30}\n"
      "    - {name: tx2, topic: /tx2, h2c: 1, source_fps: 30, output_fps: 30}\n"
      "    - {name: tx4, topic: /tx4, h2c: 2, source_fps: 10, output_fps: 30}\n");
  CHECK(config.video_backend == VideoBackend::Xdma);
  CHECK(config.strict);
  CHECK(config.video_queue_frames == 3);
  CHECK(config.video_max_lateness_us == 6000);
  CHECK(config.lidar_max_lateness_us == 21000);
  CHECK(config.lidar.max_lateness_us == 21000);
  CHECK(config.video_mcap == "/tmp/other.mcap");
}

TEST_CASE(orchestrator_rejects_non_strict_mode) {
  CHECK_THROWS(load_config_text(
      "mode:\n"
      "  strict: false\n"
      "video:\n"
      "  mcap: /tmp/video.mcap\n"
      "  cache_dir: /tmp/cache\n"));
}

TEST_CASE(orchestrator_rejects_loop_and_unsafe_queue) {
  CHECK_THROWS(load_config_text(std::string(kValidLanes) +
                                "playback:\n  loop_count: 0\n"));
  CHECK_THROWS(load_config_text(
      "video:\n"
      "  mcap: /tmp/video.mcap\n"
      "  cache_dir: /tmp/cache\n"
      "  queue_frames: 7\n"));
}

TEST_CASE(orchestrator_rejects_bad_backend_paths_and_lateness) {
  CHECK_THROWS(load_config_text(std::string(kValidLanes) +
                                "mode:\n  video_backend: hardware\n"));
  CHECK_THROWS(load_config_text(
      "video:\n  mcap: ''\n  cache_dir: /tmp/cache\n"));
  CHECK_THROWS(load_config_text(
      "video:\n  mcap: /tmp/video.mcap\n  cache_dir: ''\n"));
  CHECK_THROWS(load_config_text(std::string(kValidLanes) +
                                "timing:\n  video_max_lateness_us: 0\n"));
}

TEST_CASE(orchestrator_rejects_invalid_lane_contracts) {
  CHECK_THROWS(load_config_text(
      "video:\n"
      "  mcap: /tmp/video.mcap\n"
      "  cache_dir: /tmp/cache\n"
      "  lanes:\n"
      "    - {name: tx1, topic: /a, h2c: 0, source_fps: 10, output_fps: 30}\n"
      "    - {name: tx1, topic: /b, h2c: 1, source_fps: 30, output_fps: 30}\n"
      "    - {name: tx4, topic: /c, h2c: 2, source_fps: 10, output_fps: 30}\n"));
  CHECK_THROWS(load_config_text(
      "video:\n"
      "  mcap: /tmp/video.mcap\n"
      "  cache_dir: /tmp/cache\n"
      "  lanes:\n"
      "    - {name: tx1, topic: /a, h2c: 0, source_fps: 10, output_fps: 30}\n"
      "    - {name: tx2, topic: /b, h2c: 0, source_fps: 30, output_fps: 30}\n"
      "    - {name: tx4, topic: /c, h2c: 2, source_fps: 10, output_fps: 30}\n"));
  CHECK_THROWS(load_config_text(
      "video:\n"
      "  mcap: /tmp/video.mcap\n"
      "  cache_dir: /tmp/cache\n"
      "  lanes:\n"
      "    - {name: tx1, topic: '', h2c: 0, source_fps: 10, output_fps: 30}\n"
      "    - {name: tx2, topic: /b, h2c: 1, source_fps: 30, output_fps: 30}\n"
      "    - {name: tx4, topic: /c, h2c: 2, source_fps: 10, output_fps: 30}\n"));
  CHECK_THROWS(load_config_text(
      "video:\n"
      "  mcap: /tmp/video.mcap\n"
      "  cache_dir: /tmp/cache\n"
      "  lanes:\n"
      "    - {name: tx1, topic: /a, h2c: 0, source_fps: 15, output_fps: 30}\n"
      "    - {name: tx2, topic: /b, h2c: 1, source_fps: 30, output_fps: 30}\n"
      "    - {name: tx4, topic: /c, h2c: 2, source_fps: 10, output_fps: 30}\n"));
}

TEST_CASE(orchestrator_requires_exactly_three_lanes) {
  CHECK_THROWS(load_config_text(
      "video:\n"
      "  mcap: /tmp/video.mcap\n"
      "  cache_dir: /tmp/cache\n"
      "  lanes:\n"
      "    - {name: tx1, topic: /a, h2c: 0, source_fps: 10, output_fps: 30}\n"));
}

TEST_CASE(orchestrator_requires_all_eight_default_lidars_enabled) {
  CHECK_THROWS(load_config_text(
      std::string(kValidLanes) +
      "lidars:\n"
      "  - {slot: 31, enabled: false}\n"));
  CHECK_THROWS(load_config_text(
      std::string(kValidLanes) +
      "lidars:\n"
      "  - {slot: 31, topic: /sensor/lidar_left/orig/udp_packets}\n"));
}

TEST_CASE(orchestrator_mock_config_references_existing_real_inputs) {
  const auto config = load_orchestrator_config(
      "/home/ubuntu/zhang/joint_sensor_player/config/orchestrator_mock.yaml");
  CHECK(std::filesystem::is_regular_file(config.video_mcap));
  CHECK(std::filesystem::is_directory(config.lidar.data_dir));
  CHECK(config.cache_dir == "/data/joint_sensor_player/cache");
  CHECK(std::filesystem::exists(
      std::filesystem::path("/sys/class/net") / config.lidar.interface));
  bool has_radar_mcap = false;
  for (const auto& entry :
       std::filesystem::directory_iterator(config.lidar.data_dir)) {
    has_radar_mcap =
        has_radar_mcap ||
        (entry.is_regular_file() && entry.path().extension() == ".mcap");
  }
  CHECK(has_radar_mcap);
  CHECK(config.lidar.channels.size() == 8);
  for (const auto& channel : config.lidar.channels) {
    CHECK(channel.enabled);
    CHECK(std::filesystem::is_regular_file(channel.correction_file));
  }
}
