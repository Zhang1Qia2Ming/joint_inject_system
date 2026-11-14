// 读取三路 UYVY 缓存清单，核对帧大小、时间戳和配置里的车道是否一致。

#include "joint_sensor_orchestrator/video_cache.hpp"

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <charconv>
#include <fstream>
#include <iomanip>
#include <vector>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <system_error>

namespace joint_sensor_orchestrator {
namespace {

using Json = nlohmann::json;

[[noreturn]] void invalid(const std::string& reason) {
  throw std::runtime_error("invalid video cache: " + reason);
}

std::filesystem::path checked_cache_path(
    const std::filesystem::path& root, const std::string& name) {
  const std::filesystem::path relative(name);
  if (relative.empty() || relative.is_absolute() ||
      relative.filename() != relative) {
    invalid("cache file must be a plain relative filename");
  }
  return root / relative;
}

std::vector<std::uint64_t> read_timestamps(
    const std::filesystem::path& path, std::uint64_t expected_count) {
  std::ifstream input(path);
  if (!input) {
    invalid("cannot open timestamps file " + path.string());
  }
  std::vector<std::uint64_t> timestamps;
  std::string line;
  while (std::getline(input, line)) {
    std::uint64_t value{};
    const auto parsed =
        std::from_chars(line.data(), line.data() + line.size(), value);
    if (line.empty() || parsed.ec != std::errc{} ||
        parsed.ptr != line.data() + line.size()) {
      invalid("malformed timestamp in " + path.string());
    }
    if (!timestamps.empty() && value <= timestamps.back()) {
      invalid("timestamps are not strictly increasing in " + path.string());
    }
    timestamps.push_back(value);
  }
  if (!input.eof()) {
    invalid("failed while reading timestamps file " + path.string());
  }
  if (timestamps.size() != expected_count) {
    invalid("frame and timestamp counts differ in " + path.string());
  }
  return timestamps;
}

std::filesystem::path payload_stamp_path(const std::filesystem::path& path) {
  return std::filesystem::path(path.string() + ".checked");
}

bool payload_stamp_matches(const std::filesystem::path& path,
                           std::uint64_t size, const std::string& sha) {
  std::ifstream input(payload_stamp_path(path));
  std::uint64_t stamped_size = 0;
  std::uint64_t stamped_mtime = 0;
  std::string stamped_sha;
  input >> stamped_size >> stamped_mtime >> stamped_sha;
  if (!input || stamped_sha != sha || stamped_size != size) return false;
  struct stat status {};
  if (::stat(path.c_str(), &status) != 0) return false;
  const auto mtime = static_cast<std::uint64_t>(status.st_mtim.tv_sec) *
                         1'000'000'000ULL +
                     static_cast<std::uint64_t>(status.st_mtim.tv_nsec);
  return mtime == stamped_mtime;
}

void write_payload_stamp(const std::filesystem::path& path,
                         std::uint64_t size, const std::string& sha) {
  struct stat status {};
  if (::stat(path.c_str(), &status) != 0) return;
  const auto mtime = static_cast<std::uint64_t>(status.st_mtim.tv_sec) *
                         1'000'000'000ULL +
                     static_cast<std::uint64_t>(status.st_mtim.tv_nsec);
  const auto stamp = payload_stamp_path(path);
  const auto temporary = std::filesystem::path(stamp.string() + ".tmp");
  std::ofstream output(temporary, std::ios::trunc);
  if (!output) return;
  output << size << ' ' << mtime << ' ' << sha << '\n';
  output.close();
  if (!output) return;
  std::error_code error;
  std::filesystem::rename(temporary, stamp, error);
}

std::string sha256_file(const std::filesystem::path& path) {
  using Context =
      std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
  Context context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) !=
                      1) {
    throw std::runtime_error("cannot initialize SHA256");
  }

  const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) {
    invalid("cannot open payload " + path.string());
  }
  const struct FdCloser {
    int fd;
    ~FdCloser() { ::close(fd); }
  } closer{fd};
  // Hashing a 30 GiB UYVY must not leave the whole file in the page cache,
  // or the later RAM prefetch window is the first thing reclaimed.
  ::posix_fadvise(fd, 0, 0, POSIX_FADV_SEQUENTIAL);
  std::vector<char> buffer(1024 * 1024);
  off_t consumed = 0;
  while (true) {
    const ssize_t count = ::read(fd, buffer.data(), buffer.size());
    if (count < 0) {
      if (errno == EINTR) continue;
      invalid("failed while hashing payload " + path.string());
    }
    if (count == 0) break;
    if (EVP_DigestUpdate(context.get(), buffer.data(),
                         static_cast<std::size_t>(count)) != 1) {
      throw std::runtime_error("cannot update SHA256");
    }
    consumed += count;
    if (consumed > static_cast<off_t>(buffer.size())) {
      ::posix_fadvise(fd, 0, consumed - static_cast<off_t>(buffer.size()),
                      POSIX_FADV_DONTNEED);
    }
  }
  ::posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);

  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned digest_size{};
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1) {
    throw std::runtime_error("cannot finalize SHA256");
  }
  std::ostringstream text;
  text << std::hex << std::setfill('0');
  for (unsigned index = 0; index < digest_size; ++index) {
    text << std::setw(2) << static_cast<unsigned>(digest[index]);
  }
  return text.str();
}

std::string sha256_text(std::string_view value) {
  using Context =
      std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
  Context context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) !=
                      1 ||
      EVP_DigestUpdate(context.get(), value.data(), value.size()) != 1) {
    throw std::runtime_error("cannot calculate SHA256");
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned digest_size{};
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1) {
    throw std::runtime_error("cannot finalize SHA256");
  }
  std::ostringstream text;
  text << std::hex << std::setfill('0');
  for (unsigned index = 0; index < digest_size; ++index) {
    text << std::setw(2) << static_cast<unsigned>(digest[index]);
  }
  return text.str();
}

std::string expected_cache_key(const std::string& dataset_id,
                               const OrchestratorConfig& config) {
  Json topics = Json::array();
  for (const auto& lane : config.lanes) {
    topics.push_back(lane.topic);
  }
  const Json identity = {
      {"converter_version", kVideoCacheConverterVersion},
      {"dataset_id", dataset_id},
      {"height", 2160},
      {"pixel_format", "uyvy422"},
      {"topics", std::move(topics)},
      {"width", 3840},
  };
  return sha256_text(identity.dump());
}

Json read_manifest(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    invalid("cannot open manifest");
  }
  try {
    return Json::parse(input);
  } catch (const std::exception& error) {
    invalid(std::string("cannot parse manifest: ") + error.what());
  }
}

std::uint64_t file_mtime_ns(const std::filesystem::path& path) {
  struct stat status {};
  if (::stat(path.c_str(), &status) != 0) {
    invalid("cannot stat source MCAP");
  }
  return static_cast<std::uint64_t>(status.st_mtim.tv_sec) *
             1'000'000'000ULL +
         static_cast<std::uint64_t>(status.st_mtim.tv_nsec);
}

}  // namespace

VideoCache load_video_cache(const std::filesystem::path& cache_dir,
                            const OrchestratorConfig& expected_config,
                            std::uint64_t expected_frame_bytes) {
  if (expected_frame_bytes == 0) {
    throw std::invalid_argument("expected frame bytes must be positive");
  }
  try {
    if (std::filesystem::weakly_canonical(cache_dir.parent_path()) !=
        std::filesystem::weakly_canonical(expected_config.cache_dir)) {
      invalid("cache root does not match configuration");
    }
    const auto source_path =
        std::filesystem::canonical(expected_config.video_mcap);
    const auto source_size = std::filesystem::file_size(source_path);
    const auto dataset_id = sha256_file(source_path);
    const auto cache_key = expected_cache_key(dataset_id, expected_config);
    if (cache_dir.filename() != cache_key) {
      invalid("cache directory key mismatch");
    }

    const Json manifest = read_manifest(cache_dir / "manifest.json");
    if (manifest.at("version").get<unsigned>() != 1) {
      invalid("unsupported manifest version");
    }
    if (manifest.at("width").get<unsigned>() != 3840 ||
        manifest.at("height").get<unsigned>() != 2160 ||
        manifest.at("pixel_format").get<std::string>() != "uyvy422") {
      invalid("unexpected video format");
    }
    const auto& lane_entries = manifest.at("lanes");
    if (!lane_entries.is_object() ||
        lane_entries.size() != expected_config.lanes.size()) {
      invalid("manifest must contain exactly three lanes");
    }

    VideoCache cache;
    cache.version = manifest.at("version").get<unsigned>();
    cache.dataset_id = manifest.at("dataset_id").get<std::string>();
    cache.cache_key = manifest.at("cache_key").get<std::string>();
    if (cache.dataset_id != dataset_id || cache.cache_key != cache_key) {
      invalid("dataset or cache key mismatch");
    }
    cache.converter_version =
        manifest.at("converter_version").get<std::string>();
    if (cache.converter_version != kVideoCacheConverterVersion) {
      invalid("converter version mismatch");
    }
    const auto& source = manifest.at("source_mcap");
    cache.source_mcap_path = source.at("path").get<std::string>();
    cache.source_mcap_size = source.at("size").get<std::uint64_t>();
    cache.source_mcap_manifest_mtime_ns =
        source.at("mtime_ns").get<std::uint64_t>();
    cache.source_mcap_current_mtime_ns = file_mtime_ns(source_path);
    cache.source_metadata_changed =
        cache.source_mcap_manifest_mtime_ns !=
        cache.source_mcap_current_mtime_ns;
    if (std::filesystem::path(cache.source_mcap_path) != source_path ||
        cache.source_mcap_size != source_size ||
        source.at("sha256").get<std::string>() != dataset_id) {
      invalid("source MCAP metadata mismatch");
    }

    for (std::size_t index = 0; index < expected_config.lanes.size();
         ++index) {
      const auto& expected = expected_config.lanes[index];
      const auto& entry = lane_entries.at(expected.name);
      if (entry.at("topic").get<std::string>() != expected.topic ||
          entry.at("source_fps").get<unsigned>() != expected.source_fps ||
          entry.at("output_fps").get<unsigned>() != expected.output_fps) {
        invalid("topic or rate mismatch for lane " + expected.name);
      }
      if (entry.at("pixel_format").get<std::string>() != "uyvy422") {
        invalid("pixel format mismatch for lane " + expected.name);
      }

      VideoCacheLane lane;
      lane.name = expected.name;
      lane.topic = expected.topic;
      lane.source_fps = expected.source_fps;
      lane.output_fps = expected.output_fps;
      lane.frame_count = entry.at("frame_count").get<std::uint64_t>();
      lane.frame_bytes = entry.at("frame_bytes").get<std::uint64_t>();
      lane.sha256 = entry.at("sha256").get<std::string>();
      if (lane.frame_count == 0 ||
          lane.frame_bytes != expected_frame_bytes) {
        invalid("frame metadata mismatch for lane " + expected.name);
      }
      if (lane.frame_count >
          std::numeric_limits<std::uint64_t>::max() / lane.frame_bytes) {
        invalid("payload size overflow for lane " + expected.name);
      }

      lane.payload_path = checked_cache_path(
          cache_dir, entry.at("payload_file").get<std::string>());
      const auto& timestamps = entry.at("timestamps");
      lane.timestamps_path = checked_cache_path(
          cache_dir, timestamps.at("file").get<std::string>());
      lane.timestamps_byte_size =
          timestamps.at("byte_size").get<std::uint64_t>();
      lane.timestamps_sha256 =
          timestamps.at("sha256").get<std::string>();
      std::error_code error;
      const auto payload_size =
          std::filesystem::file_size(lane.payload_path, error);
      if (error ||
          payload_size != lane.frame_count * lane.frame_bytes) {
        invalid("payload size mismatch for lane " + expected.name);
      }
      const auto timestamps_size =
          std::filesystem::file_size(lane.timestamps_path, error);
      if (error || timestamps_size != lane.timestamps_byte_size ||
          lane.timestamps_sha256.size() != 64 ||
          sha256_file(lane.timestamps_path) != lane.timestamps_sha256) {
        invalid("timestamps integrity mismatch for lane " + expected.name);
      }
      lane.timestamps_ns =
          read_timestamps(lane.timestamps_path, lane.frame_count);
      if (lane.sha256.size() != 64) {
        invalid("payload SHA256 mismatch for lane " + expected.name);
      }
      if (!payload_stamp_matches(lane.payload_path, payload_size,
                                 lane.sha256)) {
        if (sha256_file(lane.payload_path) != lane.sha256) {
          invalid("payload SHA256 mismatch for lane " + expected.name);
        }
        write_payload_stamp(lane.payload_path, payload_size, lane.sha256);
      }
      cache.lanes[index] = std::move(lane);
    }
    return cache;
  } catch (const nlohmann::json::exception& error) {
    invalid(std::string("manifest field error: ") + error.what());
  }
}

}  // namespace joint_sensor_orchestrator
