#include "joint_sensor_orchestrator/video_ahead_cache.hpp"
#include "test_framework.hpp"

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <set>
#include <vector>

namespace {

class TinyAheadCache {
 public:
  TinyAheadCache() {
    root_ = std::filesystem::temp_directory_path() /
            ("video_ahead_" + std::to_string(next_id_++));
    std::filesystem::create_directories(root_);
    const std::vector<std::uint64_t> times{0, 100'000'000, 200'000'000,
                                           300'000'000, 400'000'000};
    for (std::size_t index = 0; index < cache_.lanes.size(); ++index) {
      auto& lane = cache_.lanes[index];
      lane.name = "lane" + std::to_string(index);
      lane.source_fps = 10;
      lane.output_fps = 30;
      lane.frame_bytes = 16;
      lane.frame_count = times.size();
      lane.timestamps_ns = times;
      lane.payload_path = root_ / (lane.name + ".uyvy");
      std::ofstream out(lane.payload_path, std::ios::binary);
      std::vector<char> bytes(16 * times.size(),
                              static_cast<char>('A' + index));
      out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }
  }

  ~TinyAheadCache() { std::filesystem::remove_all(root_); }

  const joint_sensor_orchestrator::VideoCache& cache() const { return cache_; }

  joint_sensor_orchestrator::AheadPread pread() {
    return [this](int, void* data, std::size_t size,
                  std::uint64_t offset) -> std::ptrdiff_t {
      const auto index = offset / 16;
      read_indices_.insert(index);
      auto* bytes = static_cast<std::byte*>(data);
      for (std::size_t i = 0; i < size; ++i) bytes[i] = std::byte{0x11};
      return static_cast<std::ptrdiff_t>(size);
    };
  }

  bool was_read(std::uint64_t source_index) const {
    return read_indices_.count(source_index) != 0;
  }

 private:
  inline static unsigned next_id_{};
  std::filesystem::path root_;
  joint_sensor_orchestrator::VideoCache cache_;
  std::set<std::uint64_t> read_indices_;
};

}  // namespace

TEST_CASE(video_ahead_cache_reads_only_the_prefetch_window) {
  TinyAheadCache fixture;
  joint_sensor_orchestrator::VideoAheadCache cache(
      fixture.cache(), 200'000'000ULL, fixture.pread());
  const auto keep_going = [] { return false; };
  cache.prefetch_from(0, 0, keep_going);
  CHECK(fixture.was_read(0));
  CHECK(fixture.was_read(1));
  CHECK(!fixture.was_read(2));
  CHECK(cache.resident_count(0) == 2);

  cache.release_before(0, 2);
  CHECK(cache.resident_count(0) == 0);
  cache.prefetch_from(0, 200'000'000ULL, keep_going);
  CHECK(fixture.was_read(2));
  CHECK(fixture.was_read(3));
  CHECK(!fixture.was_read(4));
  CHECK(cache.resident_count(0) == 2);

  const auto frame = cache.acquire(0, 2, keep_going);
  CHECK(frame != nullptr);
  CHECK(frame->size() == 16);
  CHECK((*frame)[0] == std::byte{0x11});
}

TEST_CASE(video_ahead_cache_splits_large_frames_into_1mib_reads) {
  const auto root = std::filesystem::temp_directory_path() /
                    "video_ahead_4mib";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  joint_sensor_orchestrator::VideoCache cached;
  constexpr std::size_t kFrameBytes = 8ULL * 1024 * 1024;
  for (std::size_t index = 0; index < cached.lanes.size(); ++index) {
    auto& lane = cached.lanes[index];
    lane.name = "lane" + std::to_string(index);
    lane.source_fps = 30;
    lane.output_fps = 30;
    lane.frame_bytes = kFrameBytes;
    lane.frame_count = 1;
    lane.timestamps_ns = {0};
    lane.payload_path = root / (lane.name + ".uyvy");
    std::ofstream out(lane.payload_path, std::ios::binary);
    std::vector<char> bytes(kFrameBytes, '\x7e');
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  }
  std::vector<std::size_t> sizes;
  joint_sensor_orchestrator::VideoAheadCache cache(
      cached, 1'000'000'000ULL,
      [&](int, void* data, std::size_t size, std::uint64_t) -> std::ptrdiff_t {
        sizes.push_back(size);
        auto* bytes = static_cast<std::byte*>(data);
        for (std::size_t i = 0; i < size; ++i) bytes[i] = std::byte{1};
        return static_cast<std::ptrdiff_t>(size);
      });
  const auto keep_going = [] { return false; };
  CHECK(cache.prefetch_one(0, 0, keep_going));
  CHECK(sizes.size() == 8);
  for (const auto size : sizes) CHECK(size == 1ULL * 1024 * 1024);
  std::filesystem::remove_all(root);
}
