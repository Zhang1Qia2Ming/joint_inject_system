#include "joint_sensor_orchestrator/shadow_video.hpp"
#include "test_framework.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

namespace {

void write_page_frames(const std::filesystem::path& path, int frames) {
  const int fd = ::open(path.c_str(), O_CREAT | O_TRUNC | O_WRONLY, 0644);
  if (fd < 0) throw std::runtime_error("open");
  std::vector<char> frame(4096, 0x5a);
  for (int index = 0; index < frames; ++index) {
    frame[0] = static_cast<char>(index);
    if (::write(fd, frame.data(), frame.size()) !=
        static_cast<ssize_t>(frame.size())) {
      ::close(fd);
      throw std::runtime_error("write");
    }
  }
  ::close(fd);
}

}  // namespace

TEST_CASE(shadow_video_reads_scheduled_frame_and_does_not_write_h2c) {
  const auto dir =
      std::filesystem::temp_directory_path() / "shadow-video-test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directory(dir);
  const auto payload = dir / "s4.uyvy";
  write_page_frames(payload, 2);
  joint_sensor_orchestrator::ShadowLaneInput lane;
  lane.name = "s4";
  lane.payload_path = payload;
  lane.timestamps_ns = {0, 90000000};
  lane.frame_bytes = 4096;
  lane.source_fps = 10;
  lane.output_fps = 30;
  joint_sensor_player::MonotonicClock clock;
  auto stop = std::make_shared<joint_sensor_player::CancellationToken>();
  joint_sensor_orchestrator::ShadowVideo shadow(
      {lane}, joint_sensor_orchestrator::SourceWindow{0, 100000000}, clock,
      stop, 1'000'000'000ULL);
  shadow.start();
  shadow.notify_play_start(
      {clock.now_ns(), 0});
  const auto result = shadow.wait();
  CHECK(result.ok);
  CHECK(result.lanes.size() == 1);
  CHECK(result.lanes[0].frames_read == 1);
  CHECK(result.lanes[0].outputs == 3);
  CHECK(result.lanes[0].h2c_writes == 0);
  std::filesystem::remove_all(dir);
}
