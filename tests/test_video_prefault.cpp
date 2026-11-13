#include "joint_sensor_orchestrator/video_prefault.hpp"
#include "test_framework.hpp"

#include <filesystem>
#include <fstream>
#include <vector>

namespace {

class TinyCache {
 public:
  TinyCache() {
    root_ = std::filesystem::temp_directory_path() /
            ("video_prefault_" + std::to_string(next_id_++));
    std::filesystem::create_directories(root_);
    build_lane(0, "tx1", 10, {0, 100'000'000, 200'000'000}, 16);
    build_lane(1, "tx2", 30, {0, 33'333'333, 66'666'666, 100'000'000}, 16);
    build_lane(2, "tx4", 10, {0, 100'000'000, 200'000'000}, 16);
  }

  ~TinyCache() { std::filesystem::remove_all(root_); }

  const joint_sensor_orchestrator::VideoCache& cache() const { return cache_; }

 private:
  void build_lane(std::size_t index, const std::string& name,
                  unsigned source_fps,
                  const std::vector<std::uint64_t>& times,
                  std::size_t frame_bytes) {
    auto& lane = cache_.lanes[index];
    lane.name = name;
    lane.topic = "/" + name;
    lane.source_fps = source_fps;
    lane.output_fps = 30;
    lane.frame_bytes = frame_bytes;
    lane.frame_count = times.size();
    lane.timestamps_ns = times;
    lane.payload_path = root_ / (name + ".uyvy");
    std::ofstream out(lane.payload_path, std::ios::binary);
    std::vector<char> bytes(frame_bytes * times.size(),
                            static_cast<char>('A' + index));
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  }

  inline static unsigned next_id_{};
  std::filesystem::path root_;
  joint_sensor_orchestrator::VideoCache cache_;
};

}  // namespace

TEST_CASE(video_prefault_reads_unique_source_frames_in_window) {
  TinyCache fixture;
  const auto frames =
      joint_sensor_orchestrator::prefault_video_playback_window(
          fixture.cache(), {0, 250'000'000}, std::nullopt);
  // tx1/tx4 source@200 needs third repeat beyond 250ms end → excluded
  // tx1:0,100 ; tx2:0,33,66,100 ; tx4:0,100
  CHECK(frames == 8);
}

TEST_CASE(video_prefault_honors_relative_duration_cap) {
  TinyCache fixture;
  const auto frames =
      joint_sensor_orchestrator::prefault_video_playback_window(
          fixture.cache(), {0, 200'000'000}, 50'000'000ULL);
  // slot_time<=50ms covers early sources only.
  CHECK(frames == 4);
}

TEST_CASE(video_prefault_pins_pages_and_optionally_dontneeds_outside) {
  TinyCache fixture;
  auto pinned_only =
      joint_sensor_orchestrator::prefault_and_pin_video_playback_window(
          fixture.cache(), {0, 200'000'000}, 50'000'000ULL,
          1ULL * 1024 * 1024, false);
  CHECK(pinned_only.frames_prefaulted() == 4);
  CHECK(pinned_only.locked_bytes() > 0);
  CHECK(pinned_only.outside_bytes_dontneed() == 0);

  auto pinned_and_dropped =
      joint_sensor_orchestrator::prefault_and_pin_video_playback_window(
          fixture.cache(), {0, 200'000'000}, 50'000'000ULL,
          1ULL * 1024 * 1024, true);
  CHECK(pinned_and_dropped.frames_prefaulted() == 4);
  CHECK(pinned_and_dropped.outside_bytes_dontneed() > 0);
}
