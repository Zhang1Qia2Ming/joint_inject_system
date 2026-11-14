// 核对外部缓存构建程序的输出，确认清单和这次配置是同一份视频。

#include "joint_sensor_orchestrator/cache_preparer.hpp"

#include <stdexcept>

namespace joint_sensor_orchestrator {

VideoCache validate_cache_builder_output(
    const OrchestratorConfig& config, const std::string& stdout_text,
    std::uint64_t expected_frame_bytes) {
  if (stdout_text.empty() || stdout_text.back() != '\n' ||
      stdout_text.find('\n') != stdout_text.size() - 1) {
    throw std::runtime_error(
        "缓存构建器必须只输出一个key目录绝对路径");
  }
  const std::filesystem::path reported(
      stdout_text.substr(0, stdout_text.size() - 1));
  if (!reported.is_absolute() || reported.filename().empty()) {
    throw std::runtime_error("缓存构建器返回的key目录无效");
  }
  const auto canonical_reported = std::filesystem::canonical(reported);
  const auto canonical_root = std::filesystem::canonical(config.cache_dir);
  if (canonical_reported.parent_path() != canonical_root) {
    throw std::runtime_error("缓存构建器返回目录不属于配置cache_dir");
  }
  return load_video_cache(
      canonical_reported, config, expected_frame_bytes);
}

}  // namespace joint_sensor_orchestrator
