#include "joint_sensor_orchestrator/orchestrator_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <set>
#include <stdexcept>

namespace joint_sensor_orchestrator {
namespace {

template <typename T>
void set_if_present(const YAML::Node& node, const char* key, T& value) {
  if (node[key]) value = node[key].as<T>();
}

void set_if_present(const YAML::Node& node, const char* key,
                    std::filesystem::path& value) {
  if (node[key]) value = node[key].as<std::string>();
}

void validate(const OrchestratorConfig& config) {
  if (!config.strict) {
    throw std::runtime_error("联合播放器第一版仅支持严格模式");
  }
  if (config.loop_count != 1) {
    throw std::runtime_error("联合播放器第一版仅支持loop_count: 1");
  }
  if (config.video_queue_frames < 3 || config.video_queue_frames > 6) {
    throw std::runtime_error("video.queue_frames必须在3到6之间");
  }
  if (config.video_max_lateness_us == 0 ||
      config.lidar_max_lateness_us == 0) {
    throw std::runtime_error("迟到门限必须大于0");
  }
  if (config.video_mcap.empty() || config.cache_dir.empty() ||
      config.lidar.data_dir.empty() || config.lidar.report_dir.empty()) {
    throw std::runtime_error("数据、缓存和报告路径不能为空");
  }
  if (config.allow_video_only_smoke &&
      config.video_backend != VideoBackend::Xdma) {
    throw std::runtime_error(
        "allow_video_only_smoke仅允许配合video_backend: xdma");
  }

  const auto default_lidars = joint_sensor_player::default_lidar_channels();
  if (config.lidar.channels.size() != default_lidars.size()) {
    throw std::runtime_error("联合播放器必须配置全部8路默认雷达");
  }
  std::set<int> lidar_slots;
  std::set<std::string> lidar_topics;
  for (const auto& expected : default_lidars) {
    const auto found = std::find_if(
        config.lidar.channels.begin(), config.lidar.channels.end(),
        [&](const auto& channel) { return channel.slot == expected.slot; });
    if (found == config.lidar.channels.end() || !found->enabled ||
        found->topic != expected.topic ||
        !lidar_slots.insert(found->slot).second ||
        !lidar_topics.insert(found->topic).second) {
      throw std::runtime_error(
          "联合播放器要求8路默认雷达全部启用且slot/topic唯一");
    }
  }

  std::set<std::string> names;
  std::set<std::string> topics;
  std::set<unsigned> h2c_indices;
  for (const auto& lane : config.lanes) {
    if (lane.name.empty() || lane.topic.empty()) {
      throw std::runtime_error("视频lane名称和topic不能为空");
    }
    if (!names.insert(lane.name).second ||
        !topics.insert(lane.topic).second ||
        !h2c_indices.insert(lane.h2c).second) {
      throw std::runtime_error("视频lane名称、topic和H2C索引必须唯一");
    }
    const bool supported_rate =
        (lane.source_fps == 10 && lane.output_fps == 30) ||
        (lane.source_fps == 30 && lane.output_fps == 30);
    if (!supported_rate) {
      throw std::runtime_error("视频帧率仅支持10到30或30到30");
    }
  }
}

}  // namespace

OrchestratorConfig load_orchestrator_config(
    const std::filesystem::path& path) {
  OrchestratorConfig config;
  config.lidar = joint_sensor_player::load_config(path);
  config.loop_count = config.lidar.loop_count;
  config.lidar_max_lateness_us = config.lidar.max_lateness_us;

  const YAML::Node root = YAML::LoadFile(path.string());
  if (const auto mode = root["mode"]) {
    set_if_present(mode, "strict", config.strict);
    set_if_present(mode, "allow_video_only_smoke",
                   config.allow_video_only_smoke);
    if (mode["video_backend"]) {
      const auto backend = mode["video_backend"].as<std::string>();
      if (backend == "mock") {
        config.video_backend = VideoBackend::Mock;
      } else if (backend == "xdma") {
        config.video_backend = VideoBackend::Xdma;
      } else {
        throw std::runtime_error("mode.video_backend必须是mock或xdma");
      }
    }
  }
  if (const auto playback = root["playback"]) {
    set_if_present(playback, "play_duration_ms", config.play_duration_ms);
    set_if_present(playback, "video_prefetch_ms", config.video_prefetch_ms);
    set_if_present(playback, "wait_for_enter", config.wait_for_enter);
  }
  if (const auto timing = root["timing"]) {
    set_if_present(timing, "video_max_lateness_us",
                   config.video_max_lateness_us);
    set_if_present(timing, "lidar_max_lateness_us",
                   config.lidar_max_lateness_us);
  }
  config.lidar.max_lateness_us = config.lidar_max_lateness_us;

  if (const auto video = root["video"]) {
    set_if_present(video, "mcap", config.video_mcap);
    set_if_present(video, "cache_dir", config.cache_dir);
    set_if_present(video, "queue_frames", config.video_queue_frames);
    if (const auto lanes = video["lanes"]) {
      if (!lanes.IsSequence() || lanes.size() != config.lanes.size()) {
        throw std::runtime_error("video.lanes必须恰好配置三路");
      }
      for (std::size_t index = 0; index < config.lanes.size(); ++index) {
        const auto lane = lanes[index];
        auto& target = config.lanes[index];
        set_if_present(lane, "name", target.name);
        set_if_present(lane, "topic", target.topic);
        set_if_present(lane, "h2c", target.h2c);
        set_if_present(lane, "source_fps", target.source_fps);
        set_if_present(lane, "output_fps", target.output_fps);
      }
    }
  }
  if (const auto shadow = root["shadow_video"]) {
    set_if_present(shadow, "manifest", config.shadow_manifest);
  }

  validate(config);
  joint_sensor_player::validate_config(config.lidar);
  return config;
}

}  // namespace joint_sensor_orchestrator
