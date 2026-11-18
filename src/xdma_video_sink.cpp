// Mock 和 XDMA 两种视频出口。XDMA 的 write_frame 只在查设备号和改计数时拿锁，
// 16 MB 的 write 在锁外面，三路可以同时进驱动。

#include "joint_sensor_orchestrator/video_sink.hpp"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <condition_variable>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace joint_sensor_orchestrator {
namespace {

struct Candidate {
    std::array<std::filesystem::path, 3> paths{};
    std::array<bool, 3> present{};
};

std::optional<unsigned> read_hex_file(const std::filesystem::path& path) {
  std::ifstream input(path);
  std::string text;
  if (!std::getline(input, text)) {
    return std::nullopt;
  }
  try {
    std::size_t parsed{};
    const unsigned value = static_cast<unsigned>(
        std::stoul(text, &parsed, 0));
    if (parsed != text.size()) {
      return std::nullopt;
    }
    return value;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

XdmaPollResult poll_writable(int device_fd, int cancel_fd, int timeout_ms) {
  std::array<pollfd, 2> descriptors{{
      {device_fd, POLLOUT, 0},
      {cancel_fd, POLLIN, 0},
  }};
  int result;
  do {
    result = ::poll(descriptors.data(), descriptors.size(), timeout_ms);
  } while (result < 0 && errno == EINTR);
  if (result == 0) {
    return XdmaPollResult::Timeout;
  }
  if (result < 0) {
    return XdmaPollResult::Error;
  }
  if ((descriptors[1].revents & POLLIN) != 0) {
    return XdmaPollResult::Cancelled;
  }
  if ((descriptors[0].revents & POLLOUT) != 0) {
    return XdmaPollResult::Ready;
  }
  return XdmaPollResult::Error;
}

template <typename Function>
class ScopeExit {
 public:
  explicit ScopeExit(Function function) : function_(std::move(function)) {}
  ScopeExit(const ScopeExit&) = delete;
  ScopeExit& operator=(const ScopeExit&) = delete;
  ~ScopeExit() noexcept { function_(); }

 private:
  Function function_;
};

template <typename Function>
ScopeExit<Function> make_scope_exit(Function function) {
  return ScopeExit<Function>(std::move(function));
}

}  // namespace

XdmaDevice discover_xdma_h2c(
    const std::filesystem::path& device_root,
    const std::filesystem::path& sysfs_class_root,
    const std::optional<std::string>& selector) {
  std::map<std::string, Candidate> candidates;
  const std::regex node_pattern(R"(^(.+)_h2c_([012])$)");
  std::error_code error;
  std::filesystem::directory_iterator entries(device_root, error);
  if (error) {
    throw std::runtime_error("无法扫描 XDMA 设备目录: " + error.message());
  }
  for (const auto& entry : entries) {
    std::smatch match;
    const std::string filename = entry.path().filename().string();
    if (!std::regex_match(filename, match, node_pattern)) {
      continue;
    }
    const std::size_t lane =
        static_cast<std::size_t>(match[2].str().front() - '0');
    auto& candidate = candidates[match[1].str()];
    candidate.paths[lane] = entry.path();
    candidate.present[lane] = true;
  }
  std::vector<XdmaDevice> valid;
  for (const auto& [name, candidate] : candidates) {
    if (selector && name != *selector) {
      continue;
    }
    if (!candidate.present[0] || !candidate.present[1] ||
        !candidate.present[2]) {
      continue;
    }
    const auto sysfs_device =
        sysfs_class_root / candidate.paths[0].filename() / "device";
    const auto vendor = read_hex_file(sysfs_device / "vendor");
    const auto device = read_hex_file(sysfs_device / "device");
    if (!vendor || *vendor != 0x10ee || !device || *device == 0 ||
        *device > std::numeric_limits<std::uint16_t>::max()) {
      continue;
    }
    valid.push_back(
        {name, static_cast<std::uint16_t>(*device), candidate.paths});
  }
  if (valid.empty()) {
    throw std::runtime_error(
        "未发现带有效 Xilinx vendor/device 信息的完整 XDMA H2C 设备");
  }
  if (valid.size() != 1) {
    throw std::runtime_error("发现多个完整 XDMA 设备，必须明确 selector");
  }
  return valid.front();
}

bool verify_xdma_gen3_x4(const std::filesystem::path& h2c0_node,
                         const std::filesystem::path& sysfs_class_root) {
  const auto device =
      sysfs_class_root / h2c0_node.filename() / "device";
  std::ifstream speed_file(device / "current_link_speed");
  std::ifstream width_file(device / "current_link_width");
  std::string speed_text;
  std::string width_text;
  if (!std::getline(speed_file, speed_text) ||
      !std::getline(width_file, width_text)) {
    return false;
  }
  try {
    const double speed_gt_per_second = std::stod(speed_text);
    const unsigned width = static_cast<unsigned>(std::stoul(width_text));
    return speed_gt_per_second >= 8.0 && width >= 4;
  } catch (const std::exception&) {
    return false;
  }
}

namespace {
std::atomic_bool g_xdma_write_stop{false};
}

void request_xdma_write_stop() noexcept {
  g_xdma_write_stop.store(true, std::memory_order_release);
}

void clear_xdma_write_stop() noexcept {
  g_xdma_write_stop.store(false, std::memory_order_release);
}

bool write_full_frame_with(const FrameWriteFunction& write,
                           const FramePollFunction& poll,
                           std::span<const std::byte> frame,
                           std::chrono::milliseconds timeout,
                           const std::atomic_bool& cancelled) {
  if (!write || !poll || frame.empty() || timeout.count() < 0) {
    return false;
  }
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (true) {
    if (cancelled.load(std::memory_order_acquire) ||
        g_xdma_write_stop.load(std::memory_order_acquire)) {
      return false;
    }
    if (std::chrono::steady_clock::now() >= end) {
      return false;
    }
    const std::ptrdiff_t written = write(frame.data(), frame.size());
    if (written == static_cast<std::ptrdiff_t>(frame.size())) {
      return true;
    }
    if (written < 0 && errno == EINTR) {
      continue;
    }
    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= end) {
        return false;
      }
      const auto remaining =
          std::chrono::duration_cast<std::chrono::milliseconds>(end - now);
      const int wait_ms =
          static_cast<int>(std::max<std::int64_t>(1, remaining.count()));
      if (poll(wait_ms) != XdmaPollResult::Ready) {
        return false;
      }
      continue;
    }
    return false;
  }
}

struct XdmaVideoSink::State {
  explicit State(XdmaVideoSinkOptions value) : options(std::move(value)) {}

  VideoSinkResult fail(std::string reason) {
    if (health.ok) {
      health.ok = false;
      health.error = reason;
    }
    return {false, std::move(reason)};
  }

  mutable std::mutex mutex;
  std::condition_variable no_active_writes;
  XdmaVideoSinkOptions options;
  std::array<int, 3> descriptors{-1, -1, -1};
  std::array<int, 2> cancel_pipe{-1, -1};
  VideoSinkHealth health;
  std::atomic_bool cancelled{false};
  std::size_t active_writes{};
  bool prepared{};
  bool stopped{};
};

XdmaVideoSink::XdmaVideoSink(XdmaVideoSinkOptions options)
    : state_(std::make_unique<State>(std::move(options))) {}

XdmaVideoSink::~XdmaVideoSink() { stop(); }

VideoSinkResult XdmaVideoSink::prepare() {
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (!state_->options.allow_hardware) {
    return state_->fail("XDMA 硬件访问未显式启用");
  }
  if (state_->prepared || state_->stopped) {
    return state_->fail("XDMA VideoSink 生命周期状态无效");
  }

  if (state_->options.io_timeout.count() < 0 ||
      state_->options.lane_h2c.size() != 3) {
    return state_->fail("XDMA lane 映射或写超时配置无效");
  }
  std::array<bool, 3> mapped_channels{};
  for (const auto& [lane, channel] : state_->options.lane_h2c) {
    if (lane.empty() || channel >= state_->descriptors.size() ||
        mapped_channels[channel]) {
      return state_->fail("XDMA lane 到 H2C 映射无效");
    }
    mapped_channels[channel] = true;
  }
  if (!mapped_channels[0] || !mapped_channels[1] ||
      !mapped_channels[2]) {
    return state_->fail("XDMA lane 映射必须覆盖 H2C0、H2C1、H2C2");
  }

  XdmaDevice device;
  try {
    device = discover_xdma_h2c(
        state_->options.device_root, state_->options.sysfs_class_root,
        state_->options.device_selector);
  } catch (const std::exception& error) {
    return state_->fail(error.what());
  }
  if (!verify_xdma_gen3_x4(device.h2c[0],
                           state_->options.sysfs_class_root)) {
    return state_->fail("XDMA PCIe 链路未达到 Gen3 x4");
  }
  if (::pipe2(state_->cancel_pipe.data(), O_CLOEXEC | O_NONBLOCK) != 0) {
    return state_->fail("无法创建 XDMA 取消唤醒管道");
  }
  const auto open_device =
      state_->options.open_device
          ? state_->options.open_device
          : XdmaOpenFunction([](const std::filesystem::path& path, int flags) {
              return ::open(path.c_str(), flags);
            });
  const auto close_device =
      state_->options.close_device
          ? state_->options.close_device
          : XdmaCloseFunction([](int descriptor) { ::close(descriptor); });
  for (std::size_t channel = 0; channel < device.h2c.size(); ++channel) {
    state_->descriptors[channel] =
        open_device(device.h2c[channel],
                    O_WRONLY | O_NONBLOCK | O_CLOEXEC);
    if (state_->descriptors[channel] < 0) {
      for (int descriptor : state_->descriptors) {
        if (descriptor >= 0) {
          close_device(descriptor);
        }
      }
      ::close(state_->cancel_pipe[0]);
      ::close(state_->cancel_pipe[1]);
      state_->cancel_pipe = {-1, -1};
      state_->descriptors = {-1, -1, -1};
      return state_->fail("无法以只写方式打开 XDMA H2C 节点");
    }
  }
  state_->prepared = true;
  return {true, {}};
}

// 锁只包住查设备号、复制函数指针和 active_writes。整帧 write 在锁外面。
VideoSinkResult XdmaVideoSink::write_frame(
    const std::string& lane, std::span<const std::byte> frame,
    const VideoSlot&, std::uint64_t) {
  int descriptor{};
  int cancel_fd{};
  XdmaWriteFunction write_device;
  XdmaPollFunction poll_device;
  std::chrono::milliseconds timeout{};
  bool active_write = false;
  const auto active_write_guard = make_scope_exit([this, &active_write] {
    if (!active_write) {
      return;
    }
    std::lock_guard<std::mutex> lock(state_->mutex);
    --state_->active_writes;
    state_->no_active_writes.notify_all();
  });
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (!state_->prepared || state_->stopped) {
      return state_->fail("XDMA VideoSink 尚未准备或已经停止");
    }
    if (!state_->health.ok) {
      return {false, state_->health.error};
    }
    const auto mapping = state_->options.lane_h2c.find(lane);
    if (mapping == state_->options.lane_h2c.end()) {
      return state_->fail("未知视频 lane: " + lane);
    }
    descriptor = state_->descriptors[mapping->second];
    cancel_fd = state_->cancel_pipe[0];
    timeout = state_->options.io_timeout;
    write_device =
        state_->options.write_device
            ? state_->options.write_device
            : XdmaWriteFunction(
                  [](int fd, const std::byte* data, std::size_t size) {
                    return static_cast<std::ptrdiff_t>(
                        ::write(fd, data, size));
                  });
    poll_device =
        state_->options.poll_device
            ? state_->options.poll_device
            : XdmaPollFunction(poll_writable);
    ++state_->active_writes;
    active_write = true;
  }

  bool complete{};
  std::string exception_error;
  try {
    complete = write_full_frame_with(
        [descriptor, &write_device](const std::byte* data, std::size_t size) {
          return write_device(descriptor, data, size);
        },
        [descriptor, cancel_fd, &poll_device](int wait_ms) {
          return poll_device(descriptor, cancel_fd, wait_ms);
        },
        frame, timeout, state_->cancelled);
  } catch (const std::exception& error) {
    exception_error = std::string("XDMA 写入回调异常: ") + error.what();
  } catch (...) {
    exception_error = "XDMA 写入回调发生未知异常";
  }

  std::lock_guard<std::mutex> lock(state_->mutex);
  if (!exception_error.empty()) {
    return state_->fail(std::move(exception_error));
  }
  if (!complete) {
    if (state_->cancelled.load(std::memory_order_acquire)) {
      return state_->fail("XDMA 帧写入已取消");
    }
    return state_->fail("XDMA 帧写入失败或发生短写");
  }
  ++state_->health.frames_written;
  state_->health.bytes_written += frame.size();
  return {true, {}};
}

VideoSinkHealth XdmaVideoSink::health() const {
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->health;
}

void XdmaVideoSink::stop() noexcept {
  XdmaCloseFunction close_device;
  std::array<int, 3> descriptors{-1, -1, -1};
  std::array<int, 2> cancel_pipe{-1, -1};
  {
    std::unique_lock<std::mutex> lock(state_->mutex);
    if (state_->stopped) {
      return;
    }
    state_->stopped = true;
    state_->cancelled.store(true, std::memory_order_release);
    if (state_->cancel_pipe[1] >= 0) {
      const std::byte wake{1};
      const ssize_t wake_result =
          ::write(state_->cancel_pipe[1], &wake, sizeof(wake));
      (void)wake_result;
    }
    state_->no_active_writes.wait(
        lock, [this] { return state_->active_writes == 0; });
    descriptors = state_->descriptors;
    state_->descriptors = {-1, -1, -1};
    cancel_pipe = state_->cancel_pipe;
    state_->cancel_pipe = {-1, -1};
    close_device =
        state_->options.close_device
            ? state_->options.close_device
            : XdmaCloseFunction(
                  [](int descriptor) { ::close(descriptor); });
  }
  for (int descriptor : descriptors) {
    if (descriptor >= 0) {
      close_device(descriptor);
    }
  }
  for (int descriptor : cancel_pipe) {
    if (descriptor >= 0) {
      ::close(descriptor);
    }
  }
}

}  // namespace joint_sensor_orchestrator
