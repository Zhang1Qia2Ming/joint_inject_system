#include "joint_sensor_orchestrator/video_sink.hpp"
#include "test_framework.hpp"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

class TemporaryDirectory {
 public:
  TemporaryDirectory()
      : path_(std::filesystem::temp_directory_path() /
              ("joint_sensor_xdma_" + std::to_string(next_id_++))) {
    std::filesystem::create_directories(path_);
  }
  ~TemporaryDirectory() { std::filesystem::remove_all(path_); }
  const std::filesystem::path& path() const { return path_; }

 private:
  inline static unsigned next_id_{};
  std::filesystem::path path_;
};

void touch(const std::filesystem::path& path) {
  std::ofstream output(path);
  output << "fixture";
}

void write_text(const std::filesystem::path& path, const std::string& text) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path);
  output << text;
}

void make_xdma_device(const TemporaryDirectory& root, const std::string& name,
                      const std::string& vendor = "0x10ee",
                      const std::string& device_id = "0x903f") {
  for (unsigned channel = 0; channel < 3; ++channel) {
    touch(root.path() /
          (name + "_h2c_" + std::to_string(channel)));
  }
  const auto device =
      root.path() / "class" / (name + "_h2c_0") / "device";
  write_text(device / "vendor", vendor + "\n");
  write_text(device / "device", device_id + "\n");
  write_text(device / "current_link_speed", "8.0 GT/s PCIe\n");
  write_text(device / "current_link_width", "4\n");
}

joint_sensor_orchestrator::VideoSlot slot(
    std::uint64_t source_index, std::uint64_t output_index,
    std::uint64_t source_time, std::uint64_t grid_index,
    std::uint64_t common_start = 1'000,
    std::uint64_t play_start = 5'000'000'000) {
  const std::uint64_t relative =
      (grid_index / 30) * 1'000'000'000ULL +
      ((grid_index % 30) * 1'000'000'000ULL) / 30;
  return {source_index, output_index, grid_index, source_time,
          common_start + relative, relative, play_start + relative};
}

joint_sensor_orchestrator::MockVideoSinkOptions mock_options(
    std::vector<joint_sensor_orchestrator::VideoSinkLane> lanes = {
        {"tx1", 10, 0}, {"tx2", 30, 1}, {"tx4", 10, 2}}) {
  return {
      .frame_bytes = 4,
      .max_lateness_ns = 5,
      .grid_origin_ns = 1'000,
      .output_fps = 30,
      .play_start_ns = 5'000'000'000,
      .lanes = std::move(lanes),
      .failures = {},
  };
}

}  // namespace

using joint_sensor_orchestrator::MockVideoSink;
using joint_sensor_orchestrator::MockFailureInjection;
using joint_sensor_orchestrator::MockFailureKind;
using joint_sensor_orchestrator::MockVideoSinkOptions;
using joint_sensor_orchestrator::VideoSinkLane;
using joint_sensor_orchestrator::XdmaDevice;
using joint_sensor_orchestrator::XdmaPollResult;
using joint_sensor_orchestrator::XdmaVideoSink;
using joint_sensor_orchestrator::XdmaVideoSinkOptions;
using joint_sensor_orchestrator::discover_xdma_h2c;
using joint_sensor_orchestrator::verify_xdma_gen3_x4;
using joint_sensor_orchestrator::write_full_frame_with;

TEST_CASE(mock_sink_accepts_global_grid_and_lane_rate_contracts) {
  MockVideoSink sink(mock_options());
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  CHECK(sink.write_frame("tx1", frame, slot(0, 0, 1'000, 0),
                         5'000'000'000).ok);
  CHECK(sink.write_frame("tx1", frame, slot(0, 1, 1'000, 1),
                         5'033'333'333)
            .ok);
  CHECK(sink.write_frame("tx1", frame, slot(0, 2, 1'000, 2),
                         5'066'666'666)
            .ok);
  CHECK(sink.write_frame("tx1", frame, slot(1, 3, 100'001'000, 3),
                         5'100'000'000)
            .ok);
  CHECK(sink.write_frame("tx2", frame, slot(0, 0, 1'000, 0),
                         5'000'000'000).ok);
  CHECK(sink.write_frame("tx2", frame, slot(1, 1, 33'334'333, 1),
                         5'033'333'333)
            .ok);
}

TEST_CASE(mock_sink_can_bind_shared_play_start_from_first_slot) {
  auto options = mock_options();
  options.play_start_ns = 0;
  MockVideoSink sink(std::move(options));
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  CHECK(sink.write_frame("tx2", frame, slot(0, 0, 1'000, 0),
                         5'000'000'000).ok);
  CHECK(sink.write_frame("tx2", frame, slot(1, 1, 33'334'333, 1),
                         5'033'333'333).ok);
}

TEST_CASE(mock_sink_rejects_unknown_lane_independently) {
  MockVideoSink sink(mock_options());
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  CHECK(!sink.write_frame("bad", frame, slot(0, 0, 1'000, 0),
                          5'000'000'000).ok);
}

TEST_CASE(mock_sink_rejects_frame_size_independently) {
  MockVideoSink sink(mock_options());
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> short_frame(3);
  CHECK(!sink.write_frame("tx1", short_frame, slot(0, 0, 1'000, 0),
                          5'000'000'000)
             .ok);
}

TEST_CASE(mock_sink_rejects_non_global_grid_independently) {
  MockVideoSink sink(mock_options());
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  auto bad = slot(0, 0, 1'000, 0);
  ++bad.deadline_ns;
  CHECK(!sink.write_frame("tx1", frame, bad, 5'000'000'000).ok);
}

TEST_CASE(mock_sink_rejects_legal_slot_with_forged_source_time) {
  MockVideoSink sink(mock_options());
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  CHECK(!sink.write_frame(
                 "tx1", frame, slot(0, 0, 100'001'000, 0),
                 5'000'000'000)
             .ok);
}

TEST_CASE(mock_sink_accepts_thirty_hz_empty_global_slot) {
  MockVideoSink sink(mock_options());
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  CHECK(sink.write_frame("tx2", frame, slot(0, 0, 1'000, 0),
                         5'000'000'000).ok);
  CHECK(sink.write_frame("tx2", frame, slot(1, 1, 66'667'666, 2),
                         5'066'666'666).ok);
}

TEST_CASE(mock_sink_accepts_ten_hz_empty_slots_between_source_groups) {
  MockVideoSink sink(mock_options());
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  CHECK(sink.write_frame("tx1", frame, slot(0, 0, 1'000, 0),
                         5'000'000'000).ok);
  CHECK(sink.write_frame("tx1", frame, slot(0, 1, 1'000, 1),
                         5'033'333'333).ok);
  CHECK(sink.write_frame("tx1", frame, slot(0, 2, 1'000, 2),
                         5'066'666'666).ok);
  CHECK(sink.write_frame("tx1", frame, slot(1, 3, 166'667'666, 5),
                         5'166'666'666).ok);
  CHECK(sink.write_frame("tx1", frame, slot(1, 4, 166'667'666, 6),
                         5'200'000'000).ok);
  CHECK(sink.write_frame("tx1", frame, slot(1, 5, 166'667'666, 7),
                         5'233'333'333).ok);
}

TEST_CASE(mock_sink_rejects_nonincreasing_global_grid) {
  MockVideoSink sink(mock_options());
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  CHECK(sink.write_frame("tx2", frame, slot(0, 0, 1'000, 0),
                         5'000'000'000).ok);
  CHECK(!sink.write_frame("tx2", frame, slot(1, 1, 33'334'333, 0),
                          5'000'000'000).ok);
}

TEST_CASE(mock_sink_rejects_wrong_ten_hz_repeat_pattern_independently) {
  MockVideoSink sink(mock_options());
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  CHECK(sink.write_frame("tx1", frame, slot(0, 0, 1'000, 0),
                         5'000'000'000).ok);
  CHECK(!sink.write_frame("tx1", frame, slot(1, 1, 33'334'333, 1),
                          5'033'333'333)
             .ok);
}

TEST_CASE(mock_sink_rejects_repeated_thirty_hz_source_independently) {
  MockVideoSink sink(mock_options());
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  CHECK(sink.write_frame("tx2", frame, slot(0, 0, 1'000, 0),
                         5'000'000'000).ok);
  CHECK(!sink.write_frame("tx2", frame, slot(0, 1, 1'000, 1),
                          5'033'333'333)
             .ok);
}

TEST_CASE(mock_sink_rejects_lateness_independently) {
  MockVideoSink sink(mock_options());
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  CHECK(!sink.write_frame("tx1", frame, slot(0, 0, 1'000, 0),
                          5'000'000'006)
             .ok);
}

TEST_CASE(mock_write_failure_targets_lane_and_output_index) {
  auto options = mock_options();
  options.failures = {
      MockFailureInjection{"tx2", 1, MockFailureKind::WriteFailure},
  };
  MockVideoSink sink(std::move(options));
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  CHECK(sink.write_frame("tx1", frame, slot(0, 0, 1'000, 0),
                         5'000'000'000).ok);
  CHECK(sink.write_frame("tx1", frame, slot(0, 1, 1'000, 1),
                         5'033'333'333)
            .ok);
  CHECK(sink.write_frame("tx2", frame, slot(0, 0, 1'000, 0),
                         5'000'000'000).ok);
  CHECK(!sink.write_frame("tx2", frame, slot(1, 1, 33'334'333, 1),
                          5'033'333'333)
             .ok);
}

TEST_CASE(mock_timeout_targets_lane_and_output_index) {
  auto options = mock_options();
  options.failures = {
      MockFailureInjection{"tx2", 0, MockFailureKind::Timeout},
  };
  MockVideoSink sink(std::move(options));
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  CHECK(!sink.write_frame("tx2", frame, slot(0, 0, 1'000, 0),
                          5'000'000'000).ok);
}

TEST_CASE(mock_short_write_targets_lane_and_output_index) {
  auto options = mock_options();
  options.failures = {
      MockFailureInjection{"tx2", 0, MockFailureKind::ShortWrite},
  };
  MockVideoSink sink(std::move(options));
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  CHECK(!sink.write_frame("tx2", frame, slot(0, 0, 1'000, 0),
                          5'000'000'000).ok);
}

TEST_CASE(xdma_discovery_requires_vendor_device_and_same_three_lanes) {
  TemporaryDirectory root;
  make_xdma_device(root, "xdma0", "0x1234");
  CHECK_THROWS(discover_xdma_h2c(
      root.path(), root.path() / "class", std::nullopt));
  write_text(root.path() / "class/xdma0_h2c_0/device/vendor", "0x10ee\n");
  std::filesystem::remove(
      root.path() / "class/xdma0_h2c_0/device/device");
  CHECK_THROWS(discover_xdma_h2c(
      root.path(), root.path() / "class", std::nullopt));
  write_text(root.path() / "class/xdma0_h2c_0/device/device", "0x903f\n");
  const XdmaDevice device = discover_xdma_h2c(
      root.path(), root.path() / "class", std::nullopt);
  CHECK(device.selector == "xdma0");
  CHECK(device.device_id == 0x903f);
}

TEST_CASE(xdma_discovery_rejects_ambiguous_devices_without_selector) {
  TemporaryDirectory root;
  make_xdma_device(root, "xdma0");
  make_xdma_device(root, "xdma1");
  CHECK_THROWS(discover_xdma_h2c(
      root.path(), root.path() / "class", std::nullopt));
  const auto selected = discover_xdma_h2c(
      root.path(), root.path() / "class", std::string("xdma1"));
  CHECK(selected.selector == "xdma1");
}

TEST_CASE(xdma_frame_write_retries_eintr_and_rejects_positive_short_write) {
  const std::vector<std::byte> frame(16);
  std::atomic_bool cancelled{false};
  unsigned calls = 0;
  CHECK(write_full_frame_with(
      [&](const std::byte*, std::size_t size) -> std::ptrdiff_t {
        ++calls;
        if (calls == 1) {
          errno = EINTR;
          return -1;
        }
        return static_cast<std::ptrdiff_t>(size);
      },
      [](int) { return XdmaPollResult::Ready; }, frame,
      std::chrono::milliseconds(10), cancelled));
  CHECK(calls == 2);

  CHECK(!write_full_frame_with(
      [](const std::byte*, std::size_t size) -> std::ptrdiff_t {
        return static_cast<std::ptrdiff_t>(size - 1);
      },
      [](int) { return XdmaPollResult::Ready; }, frame,
      std::chrono::milliseconds(10), cancelled));
}

TEST_CASE(xdma_eintr_retries_still_obey_overall_deadline) {
  const std::vector<std::byte> frame(16);
  std::atomic_bool cancelled{false};
  unsigned calls = 0;
  CHECK(!write_full_frame_with(
      [&](const std::byte*, std::size_t size) -> std::ptrdiff_t {
        if (++calls == 1) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          errno = EINTR;
          return -1;
        }
        return static_cast<std::ptrdiff_t>(size);
      },
      [](int) { return XdmaPollResult::Ready; }, frame,
      std::chrono::milliseconds(1), cancelled));
  CHECK(calls == 1);
}

TEST_CASE(xdma_frame_write_waits_on_eagain_and_times_out) {
  const std::vector<std::byte> frame(16);
  std::atomic_bool cancelled{false};
  CHECK(!write_full_frame_with(
      [](const std::byte*, std::size_t) -> std::ptrdiff_t {
        errno = EAGAIN;
        return -1;
      },
      [](int) { return XdmaPollResult::Timeout; }, frame,
      std::chrono::milliseconds(5), cancelled));
}

TEST_CASE(xdma_frame_write_waits_on_eagain_then_writes_complete_frame) {
  const std::vector<std::byte> frame(16);
  std::atomic_bool cancelled{false};
  unsigned writes = 0;
  unsigned polls = 0;
  CHECK(write_full_frame_with(
      [&](const std::byte*, std::size_t size) -> std::ptrdiff_t {
        if (++writes == 1) {
          errno = EAGAIN;
          return -1;
        }
        return static_cast<std::ptrdiff_t>(size);
      },
      [&](int) {
        ++polls;
        return XdmaPollResult::Ready;
      },
      frame, std::chrono::milliseconds(20), cancelled));
  CHECK(writes == 2);
  CHECK(polls == 1);
}

TEST_CASE(xdma_link_check_uses_discovered_node_not_a_hardcoded_bdf) {
  TemporaryDirectory root;
  make_xdma_device(root, "xdma7");
  const auto class_root = root.path() / "class";
  CHECK(verify_xdma_gen3_x4(root.path() / "xdma7_h2c_0", class_root));

  write_text(class_root / "xdma7_h2c_0/device/current_link_speed",
             "5.0 GT/s PCIe\n");
  CHECK(!verify_xdma_gen3_x4(root.path() / "xdma7_h2c_0", class_root));
}

TEST_CASE(xdma_prepare_requires_exactly_three_unique_complete_lane_mappings) {
  unsigned opens = 0;
  XdmaVideoSink missing(XdmaVideoSinkOptions{
      .allow_hardware = true,
      .device_root = "/unused",
      .sysfs_class_root = "/unused",
      .device_selector = std::nullopt,
      .io_timeout = std::chrono::milliseconds(20),
      .lane_h2c = {{"tx1", 0}, {"tx2", 1}},
      .open_device = [&](const std::filesystem::path&, int) {
        ++opens;
        return -1;
      },
      .write_device = {},
      .poll_device = {},
      .close_device = {}});
  CHECK(!missing.prepare().ok);
  CHECK(opens == 0);

  XdmaVideoSink duplicate(XdmaVideoSinkOptions{
      .allow_hardware = true,
      .device_root = "/unused",
      .sysfs_class_root = "/unused",
      .device_selector = std::nullopt,
      .io_timeout = std::chrono::milliseconds(20),
      .lane_h2c = {{"tx1", 0}, {"tx2", 0}, {"tx4", 2}},
      .open_device = [&](const std::filesystem::path&, int) {
        ++opens;
        return -1;
      },
      .write_device = {},
      .poll_device = {},
      .close_device = {}});
  CHECK(!duplicate.prepare().ok);
  CHECK(opens == 0);
}

TEST_CASE(xdma_sink_uses_configured_lane_mapping_and_nonblocking_open) {
  TemporaryDirectory root;
  make_xdma_device(root, "xdma0");
  std::unordered_map<std::string, int> opened;
  int flags_seen = 0;
  XdmaVideoSink sink(XdmaVideoSinkOptions{
      .allow_hardware = true,
      .device_root = root.path(),
      .sysfs_class_root = root.path() / "class",
      .device_selector = std::nullopt,
      .io_timeout = std::chrono::milliseconds(20),
      .lane_h2c = {{"tx4", 0}, {"tx1", 2}, {"tx2", 1}},
      .open_device = [&](const std::filesystem::path& path, int flags) {
        flags_seen = flags;
        const int fd = 100 + path.filename().string().back() - '0';
        opened[path.filename().string()] = fd;
        return fd;
      },
      .write_device = [](int fd, const std::byte*, std::size_t size) {
        CHECK(fd == 102);
        return static_cast<std::ptrdiff_t>(size);
      },
      .poll_device = {},
      .close_device = [](int) {}});
  CHECK(sink.prepare().ok);
  CHECK((flags_seen & O_ACCMODE) == O_WRONLY);
  CHECK((flags_seen & O_NONBLOCK) != 0);
  const std::vector<std::byte> frame(4);
  CHECK(sink.write_frame("tx1", frame, slot(0, 0, 1'000, 0),
                         5'000'000'000).ok);
  sink.stop();
}

TEST_CASE(xdma_sink_stop_wakes_blocked_write_without_mutex_deadlock) {
  TemporaryDirectory root;
  make_xdma_device(root, "xdma0");
  std::promise<void> polling;
  auto polling_started = polling.get_future();
  std::atomic_bool announced{false};
  XdmaVideoSink sink(XdmaVideoSinkOptions{
      .allow_hardware = true,
      .device_root = root.path(),
      .sysfs_class_root = root.path() / "class",
      .device_selector = std::nullopt,
      .io_timeout = std::chrono::seconds(5),
      .lane_h2c = {{"tx1", 0}, {"tx2", 1}, {"tx4", 2}},
      .open_device = [](const std::filesystem::path&, int) { return 100; },
      .write_device = [](int, const std::byte*, std::size_t) {
        errno = EAGAIN;
        return static_cast<std::ptrdiff_t>(-1);
      },
      .poll_device =
          [&](int, int cancel_fd, int timeout_ms) {
            if (!announced.exchange(true)) {
              polling.set_value();
            }
            pollfd descriptor{cancel_fd, POLLIN, 0};
            const int result = ::poll(&descriptor, 1, timeout_ms);
            return result > 0 ? XdmaPollResult::Cancelled
                              : XdmaPollResult::Timeout;
          },
      .close_device = [](int) {}});
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  auto write = std::async(std::launch::async, [&] {
    return sink.write_frame("tx1", frame, slot(0, 0, 1'000, 0),
                            5'000'000'000);
  });
  CHECK(polling_started.wait_for(std::chrono::milliseconds(100)) ==
        std::future_status::ready);
  sink.stop();
  CHECK(write.wait_for(std::chrono::milliseconds(100)) ==
        std::future_status::ready);
  CHECK(!write.get().ok);
}

TEST_CASE(xdma_sink_write_exception_releases_active_write_before_stop) {
  TemporaryDirectory root;
  make_xdma_device(root, "xdma0");
  std::promise<void> entered;
  auto entered_future = entered.get_future();
  std::promise<void> release;
  auto release_future = release.get_future().share();
  XdmaVideoSink sink(XdmaVideoSinkOptions{
      .allow_hardware = true,
      .device_root = root.path(),
      .sysfs_class_root = root.path() / "class",
      .device_selector = std::nullopt,
      .io_timeout = std::chrono::seconds(5),
      .lane_h2c = {{"tx1", 0}, {"tx2", 1}, {"tx4", 2}},
      .open_device = [](const std::filesystem::path&, int) { return 100; },
      .write_device = [&](int, const std::byte*, std::size_t)
          -> std::ptrdiff_t {
        entered.set_value();
        release_future.wait();
        throw std::runtime_error("injected write exception");
      },
      .poll_device = {},
      .close_device = [](int) {}});
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  auto write = std::async(std::launch::async, [&] {
    return sink.write_frame("tx1", frame, slot(0, 0, 1'000, 0),
                            5'000'000'000);
  });
  CHECK(entered_future.wait_for(std::chrono::milliseconds(100)) ==
        std::future_status::ready);
  auto stop = std::async(std::launch::async, [&] { sink.stop(); });
  release.set_value();
  CHECK(!write.get().ok);
  CHECK(stop.wait_for(std::chrono::milliseconds(100)) ==
        std::future_status::ready);
}

TEST_CASE(xdma_sink_poll_exception_releases_active_write_before_stop) {
  TemporaryDirectory root;
  make_xdma_device(root, "xdma0");
  std::promise<void> entered;
  auto entered_future = entered.get_future();
  std::promise<void> release;
  auto release_future = release.get_future().share();
  XdmaVideoSink sink(XdmaVideoSinkOptions{
      .allow_hardware = true,
      .device_root = root.path(),
      .sysfs_class_root = root.path() / "class",
      .device_selector = std::nullopt,
      .io_timeout = std::chrono::seconds(5),
      .lane_h2c = {{"tx1", 0}, {"tx2", 1}, {"tx4", 2}},
      .open_device = [](const std::filesystem::path&, int) { return 100; },
      .write_device = [](int, const std::byte*, std::size_t) {
        errno = EAGAIN;
        return static_cast<std::ptrdiff_t>(-1);
      },
      .poll_device = [&](int, int, int) -> XdmaPollResult {
        entered.set_value();
        release_future.wait();
        throw std::runtime_error("injected poll exception");
      },
      .close_device = [](int) {}});
  CHECK(sink.prepare().ok);
  const std::vector<std::byte> frame(4);
  auto write = std::async(std::launch::async, [&] {
    return sink.write_frame("tx1", frame, slot(0, 0, 1'000, 0),
                            5'000'000'000);
  });
  CHECK(entered_future.wait_for(std::chrono::milliseconds(100)) ==
        std::future_status::ready);
  auto stop = std::async(std::launch::async, [&] { sink.stop(); });
  release.set_value();
  CHECK(!write.get().ok);
  CHECK(stop.wait_for(std::chrono::milliseconds(100)) ==
        std::future_status::ready);
}

TEST_CASE(xdma_sink_default_prepare_never_opens_hardware) {
  unsigned opens = 0;
  XdmaVideoSink sink(XdmaVideoSinkOptions{
      .allow_hardware = false,
      .device_root = "/dev",
      .sysfs_class_root = "/sys/class/xdma",
      .device_selector = std::nullopt,
      .io_timeout = std::chrono::milliseconds(100),
      .lane_h2c = {},
      .open_device = [&](const std::filesystem::path&, int) {
        ++opens;
        return -1;
      },
      .write_device = {},
      .poll_device = {},
      .close_device = {}});
  CHECK(!sink.prepare().ok);
  CHECK(opens == 0);
}
