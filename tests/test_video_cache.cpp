#include "joint_sensor_orchestrator/video_cache.hpp"
#include "joint_sensor_orchestrator/cache_preparer.hpp"
#include "test_framework.hpp"

#include <nlohmann/json.hpp>
#include <sys/stat.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

class TemporaryCache {
 public:
  TemporaryCache()
      : root_(std::filesystem::temp_directory_path() /
              ("joint_sensor_video_cache_" + std::to_string(next_id_++))) {
    std::filesystem::create_directories(root_);
  }

  ~TemporaryCache() { std::filesystem::remove_all(root_); }

  const std::filesystem::path& root() const { return root_; }

 private:
  inline static unsigned next_id_{};
  std::filesystem::path root_;
};

void write_file(const std::filesystem::path& path, const std::string& data) {
  std::ofstream output(path, std::ios::binary);
  output.write(data.data(), static_cast<std::streamsize>(data.size()));
}

constexpr const char* kDatasetId =
    "76914d4d897ef68a8345628d16c79bfde0cbbd97b7f8d1fca88deaf70b673408";
constexpr const char* kCacheKey =
    "7776c192e63a30cc0dac445cd1f923d3ffb2ebbb99c8586ade7161e9779bf613";

std::uint64_t mtime_ns(const std::filesystem::path& path) {
  struct stat status {};
  if (::stat(path.c_str(), &status) != 0) {
    throw std::runtime_error("stat failed");
  }
  return static_cast<std::uint64_t>(status.st_mtim.tv_sec) * 1'000'000'000ULL +
         static_cast<std::uint64_t>(status.st_mtim.tv_nsec);
}

std::filesystem::path write_valid_cache(const std::filesystem::path& root) {
  constexpr const char* kSha256OfEightAs =
      "c34ab6abb7b2bb595bc25c3b388c872f"
      "d1d575819a8f55cc689510285e212385";
  constexpr const char* kTimestampsSha256 =
      "50a63b12f7366e16b0f4a4d68b31641d"
      "e1f5f74b5ceb30b836324729beb5ca3c";
  const auto source = root / "video.mcap";
  write_file(source, "fixture-mcap");
  const auto cache_dir = root / kCacheKey;
  std::filesystem::create_directories(cache_dir);
  for (const char* name : {"tx1", "tx2", "tx4"}) {
    write_file(cache_dir / (std::string(name) + ".uyvy"), "AAAAAAAA");
    write_file(cache_dir / (std::string(name) + ".timestamps_ns"), "100\n200\n");
  }
  nlohmann::json manifest = {
      {"version", 1},
      {"dataset_id", kDatasetId},
      {"cache_key", kCacheKey},
      {"converter_version", "joint-sensor-video-cache-1"},
      {"source_mcap",
       {{"path", std::filesystem::canonical(source).string()},
        {"size", std::filesystem::file_size(source)},
        {"mtime_ns", mtime_ns(source)},
        {"sha256", kDatasetId}}},
      {"width", 3840},
      {"height", 2160},
      {"pixel_format", "uyvy422"},
      {"lanes", nlohmann::json::object()}};
  const std::array<unsigned, 3> source_rates{10, 30, 10};
  const std::array<const char*, 3> names{"tx1", "tx2", "tx4"};
  for (std::size_t index = 0; index < names.size(); ++index) {
    const std::string name(names[index]);
    manifest["lanes"][name] = {
        {"topic", "/camera/" + name + "/h265"},
        {"source_fps", source_rates[index]},
        {"output_fps", 30},
        {"pixel_format", "uyvy422"},
        {"frame_count", 2},
        {"frame_bytes", 4},
        {"payload_file", name + ".uyvy"},
        {"timestamps",
         {{"file", name + ".timestamps_ns"},
          {"byte_size", 8},
          {"sha256", kTimestampsSha256}}},
        {"sha256", kSha256OfEightAs}};
  }
  write_file(cache_dir / "manifest.json", manifest.dump(2));
  return cache_dir;
}

joint_sensor_orchestrator::OrchestratorConfig expected_config(
    const std::filesystem::path& root) {
  joint_sensor_orchestrator::OrchestratorConfig config;
  config.video_mcap = root / "video.mcap";
  config.cache_dir = root;
  return config;
}

}  // namespace

using joint_sensor_orchestrator::load_video_cache;

TEST_CASE(video_cache_loads_only_metadata_and_paths) {
  TemporaryCache cache_root;
  const auto key_dir = write_valid_cache(cache_root.root());

  const auto cache =
      load_video_cache(key_dir, expected_config(cache_root.root()), 4);

  CHECK(cache.version == 1);
  CHECK(cache.dataset_id == kDatasetId);
  CHECK(cache.cache_key == kCacheKey);
  CHECK(cache.source_mcap_size == 12);
  CHECK(cache.source_mcap_manifest_mtime_ns ==
        mtime_ns(cache_root.root() / "video.mcap"));
  CHECK(!cache.source_metadata_changed);
  CHECK(cache.lanes[0].name == "tx1");
  CHECK(cache.lanes[0].frame_count == 2);
  CHECK(cache.lanes[0].timestamps_ns.size() == 2);
  CHECK(cache.lanes[0].timestamps_ns[1] == 200);
  CHECK(cache.lanes[0].payload_path == key_dir / "tx1.uyvy");
}

TEST_CASE(video_cache_rejects_truncated_or_changed_payload) {
  TemporaryCache cache_root;
  auto key_dir = write_valid_cache(cache_root.root());
  CHECK(load_video_cache(key_dir, expected_config(cache_root.root()), 4)
            .lanes[0]
            .frame_count == 2);

  std::filesystem::resize_file(key_dir / "tx1.uyvy", 7);
  CHECK_THROWS(
      load_video_cache(key_dir, expected_config(cache_root.root()), 4));

  key_dir = write_valid_cache(cache_root.root());
  write_file(key_dir / "tx1.uyvy", "BBBBBBBB");
  CHECK_THROWS(
      load_video_cache(key_dir, expected_config(cache_root.root()), 4));
}

TEST_CASE(video_cache_rejects_timestamp_count_and_order_errors) {
  TemporaryCache cache_root;
  auto key_dir = write_valid_cache(cache_root.root());
  write_file(key_dir / "tx2.timestamps_ns", "100\n100\n");
  CHECK_THROWS(
      load_video_cache(key_dir, expected_config(cache_root.root()), 4));

  key_dir = write_valid_cache(cache_root.root());
  write_file(key_dir / "tx2.timestamps_ns", "100\n");
  CHECK_THROWS(
      load_video_cache(key_dir, expected_config(cache_root.root()), 4));
}

TEST_CASE(video_cache_rejects_topic_rate_and_format_mismatch) {
  TemporaryCache cache_root;
  auto key_dir = write_valid_cache(cache_root.root());
  auto config = expected_config(cache_root.root());
  config.lanes[0].topic = "/different";
  CHECK_THROWS(load_video_cache(key_dir, config, 4));

  key_dir = write_valid_cache(cache_root.root());
  config = expected_config(cache_root.root());
  config.lanes[1].source_fps = 10;
  CHECK_THROWS(load_video_cache(key_dir, config, 4));

  key_dir = write_valid_cache(cache_root.root());
  auto manifest_path = key_dir / "manifest.json";
  std::ifstream input(manifest_path);
  std::string manifest((std::istreambuf_iterator<char>(input)),
                       std::istreambuf_iterator<char>());
  const auto position = manifest.find("\"uyvy422\"");
  manifest.replace(position, std::string("\"uyvy422\"").size(), "\"nv12\"");
  write_file(manifest_path, manifest);
  CHECK_THROWS(
      load_video_cache(key_dir, expected_config(cache_root.root()), 4));
}

TEST_CASE(video_cache_rejects_converter_version_mismatch) {
  TemporaryCache cache_root;
  const auto key_dir = write_valid_cache(cache_root.root());
  const auto manifest_path = key_dir / "manifest.json";
  std::ifstream input(manifest_path);
  std::string manifest((std::istreambuf_iterator<char>(input)),
                       std::istreambuf_iterator<char>());
  const auto position = manifest.find("joint-sensor-video-cache-1");
  manifest.replace(position, std::string("joint-sensor-video-cache-1").size(),
                   "stale-converter");
  write_file(manifest_path, manifest);
  CHECK_THROWS(
      load_video_cache(key_dir, expected_config(cache_root.root()), 4));
}

TEST_CASE(video_cache_rejects_source_mcap_content_change_even_same_size_mtime) {
  TemporaryCache cache_root;
  const auto key_dir = write_valid_cache(cache_root.root());
  const auto source = cache_root.root() / "video.mcap";
  const auto original_time = std::filesystem::last_write_time(source);
  write_file(source, "changed-mcap");
  std::filesystem::last_write_time(source, original_time);
  CHECK(std::filesystem::file_size(source) == 12);
  CHECK_THROWS(
      load_video_cache(key_dir, expected_config(cache_root.root()), 4));
}

TEST_CASE(video_cache_accepts_source_mtime_change_when_content_is_identical) {
  TemporaryCache cache_root;
  const auto key_dir = write_valid_cache(cache_root.root());
  const auto source = cache_root.root() / "video.mcap";
  std::filesystem::last_write_time(
      source, std::filesystem::last_write_time(source) + std::chrono::seconds(1));
  const auto cache =
      load_video_cache(key_dir, expected_config(cache_root.root()), 4);
  CHECK(cache.dataset_id == kDatasetId);
  CHECK(cache.source_metadata_changed);
  CHECK(cache.source_mcap_manifest_mtime_ns !=
        cache.source_mcap_current_mtime_ns);
}

TEST_CASE(video_cache_rejects_timestamp_content_tampering) {
  TemporaryCache cache_root;
  const auto key_dir = write_valid_cache(cache_root.root());
  const auto timestamps = key_dir / "tx1.timestamps_ns";
  write_file(timestamps, "100\n201\n");
  CHECK(std::filesystem::file_size(timestamps) == 8);
  CHECK_THROWS(
      load_video_cache(key_dir, expected_config(cache_root.root()), 4));
}

TEST_CASE(video_cache_rejects_cache_root_configuration_mismatch) {
  TemporaryCache cache_root;
  const auto key_dir = write_valid_cache(cache_root.root());
  auto config = expected_config(cache_root.root());
  config.cache_dir = cache_root.root() / "other-root";
  CHECK_THROWS(load_video_cache(key_dir, config, 4));
}

TEST_CASE(prepare_cache_output_parses_exact_key_directory_and_revalidates_manifest) {
  TemporaryCache cache_root;
  const auto key_dir = write_valid_cache(cache_root.root());
  const auto config = expected_config(cache_root.root());
  const auto cache =
      joint_sensor_orchestrator::validate_cache_builder_output(
          config, key_dir.string() + "\n", 4);
  CHECK(cache.cache_key == kCacheKey);

  write_file(key_dir / "tx1.uyvy", "BBBBBBBB");
  CHECK_THROWS(joint_sensor_orchestrator::validate_cache_builder_output(
      config, key_dir.string() + "\n", 4));
  CHECK_THROWS(joint_sensor_orchestrator::validate_cache_builder_output(
      config, key_dir.string() + "\nextra\n", 4));
}
