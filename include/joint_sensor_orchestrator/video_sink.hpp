#pragma once

#include "joint_sensor_orchestrator/unified_timeline.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace joint_sensor_orchestrator {

struct VideoSinkResult {
  bool ok{};
  std::string error;
};

struct VideoSinkHealth {
  bool ok{true};
  std::string error;
  std::uint64_t frames_written{};
  std::uint64_t bytes_written{};
  std::uint64_t max_lateness_ns{};
};

class VideoSink {
 public:
  virtual ~VideoSink() = default;
  virtual VideoSinkResult prepare() = 0;
  virtual VideoSinkResult write_frame(
      const std::string& lane, std::span<const std::byte> frame,
      const VideoSlot& slot, std::uint64_t actual_write_ns) = 0;
  virtual VideoSinkHealth health() const = 0;
  virtual void stop() noexcept = 0;
};

struct VideoSinkLane {
  std::string name;
  unsigned source_fps{};
  unsigned h2c{};
};

enum class MockFailureKind {
  WriteFailure,
  Timeout,
  ShortWrite,
};

struct MockFailureInjection {
  std::string lane;
  std::uint64_t output_index{};
  MockFailureKind kind{MockFailureKind::WriteFailure};
};

struct MockVideoSinkOptions {
  std::size_t frame_bytes{};
  std::uint64_t max_lateness_ns{};
  std::uint64_t grid_origin_ns{};
  unsigned output_fps{30};
  std::uint64_t play_start_ns{};
  std::vector<VideoSinkLane> lanes;
  std::vector<MockFailureInjection> failures;
};

class MockVideoSink final : public VideoSink {
 public:
  explicit MockVideoSink(MockVideoSinkOptions options);
  ~MockVideoSink() override;

  VideoSinkResult prepare() override;
  VideoSinkResult write_frame(
      const std::string& lane, std::span<const std::byte> frame,
      const VideoSlot& slot, std::uint64_t actual_write_ns) override;
  VideoSinkHealth health() const override;
  void stop() noexcept override;

 private:
  struct State;
  std::unique_ptr<State> state_;
};

using XdmaOpenFunction =
    std::function<int(const std::filesystem::path&, int flags)>;
using XdmaWriteFunction =
    std::function<std::ptrdiff_t(int, const std::byte*, std::size_t)>;
using XdmaCloseFunction = std::function<void(int)>;

enum class XdmaPollResult {
  Ready,
  Timeout,
  Cancelled,
  Error,
};

using XdmaPollFunction =
    std::function<XdmaPollResult(int device_fd, int cancel_fd,
                                 int timeout_ms)>;

struct XdmaVideoSinkOptions {
  bool allow_hardware{false};
  std::filesystem::path device_root{"/dev"};
  std::filesystem::path sysfs_class_root{"/sys/class/xdma"};
  std::optional<std::string> device_selector;
  std::chrono::milliseconds io_timeout{1000};
  std::unordered_map<std::string, unsigned> lane_h2c;
  XdmaOpenFunction open_device;
  XdmaWriteFunction write_device;
  XdmaPollFunction poll_device;
  XdmaCloseFunction close_device;
};

class XdmaVideoSink final : public VideoSink {
 public:
  explicit XdmaVideoSink(XdmaVideoSinkOptions options = {});
  ~XdmaVideoSink() override;

  VideoSinkResult prepare() override;
  VideoSinkResult write_frame(
      const std::string& lane, std::span<const std::byte> frame,
      const VideoSlot& slot, std::uint64_t actual_write_ns) override;
  VideoSinkHealth health() const override;
  void stop() noexcept override;

 private:
  struct State;
  std::unique_ptr<State> state_;
};

struct XdmaDevice {
  std::string selector;
  std::uint16_t device_id{};
  std::array<std::filesystem::path, 3> h2c;
};

XdmaDevice discover_xdma_h2c(
    const std::filesystem::path& device_root,
    const std::filesystem::path& sysfs_class_root,
    const std::optional<std::string>& selector);

bool verify_xdma_gen3_x4(const std::filesystem::path& h2c0_node,
                         const std::filesystem::path& sysfs_class_root);

using FrameWriteFunction =
    std::function<std::ptrdiff_t(const std::byte*, std::size_t)>;
using FramePollFunction = std::function<XdmaPollResult(int timeout_ms)>;

bool write_full_frame_with(const FrameWriteFunction& write,
                           const FramePollFunction& poll,
                           std::span<const std::byte> frame,
                           std::chrono::milliseconds timeout,
                           const std::atomic_bool& cancelled);

}  // namespace joint_sensor_orchestrator
