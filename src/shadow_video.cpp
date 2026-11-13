#include "joint_sensor_orchestrator/shadow_video.hpp"

#include "joint_sensor_orchestrator/frame_buffer_pool.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <nlohmann/json.hpp>

namespace joint_sensor_orchestrator {
namespace {

constexpr std::size_t kPreadChunkBytes = 1ULL * 1024 * 1024;

std::uint64_t scheduled_end_index(const ShadowLaneInput& lane,
                                  const SourceWindow& window) {
  const auto slots = build_video_slots(
      lane.timestamps_ns, lane.source_fps, lane.output_fps, window, 0);
  return slots.empty() ? 0 : slots.back().source_index + 1;
}

bool read_frame(int fd, std::uint64_t index, VideoFrameBuffer& buffer,
                const std::function<bool()>& should_stop) {
  const std::uint64_t offset = index * buffer.size();
  std::size_t got = 0;
  while (got < buffer.size()) {
    if (should_stop()) return false;
    const auto chunk = std::min(kPreadChunkBytes, buffer.size() - got);
    ssize_t count = 0;
    do {
      count = ::pread(fd, buffer.data() + got, chunk, offset + got);
    } while (count < 0 && errno == EINTR);
    if (count < 0 || static_cast<std::size_t>(count) != chunk) {
      throw std::runtime_error("环视模拟pread未读取完整帧");
    }
    got += static_cast<std::size_t>(count);
    if (got < buffer.size()) std::this_thread::yield();
  }
  return true;
}

}  // namespace

struct ShadowVideo::State {
  std::vector<ShadowLaneInput> lanes;
  SourceWindow window;
  joint_sensor_player::Clock* clock{};
  std::shared_ptr<joint_sensor_player::CancellationToken> external_stop;
  std::shared_ptr<joint_sensor_player::CancellationToken> local_stop =
      std::make_shared<joint_sensor_player::CancellationToken>();
  std::uint64_t prefetch_ns{};
  std::mutex mutex;
  std::condition_variable ready;
  bool play_started{false};
  StartAnchors anchors{};
  std::vector<std::thread> threads;
  ShadowVideoResult result;
  bool joined{false};
};

ShadowVideo::ShadowVideo(
    std::vector<ShadowLaneInput> lanes, SourceWindow window,
    joint_sensor_player::Clock& clock,
    std::shared_ptr<joint_sensor_player::CancellationToken> stop,
    std::uint64_t prefetch_ns)
    : state_(std::make_unique<State>()) {
  if (lanes.empty()) {
    throw std::invalid_argument("环视模拟至少需要一路");
  }
  if (!stop) throw std::invalid_argument("环视模拟需要停止令牌");
  if (prefetch_ns == 0) {
    throw std::invalid_argument("环视模拟预读窗口必须大于0");
  }
  state_->lanes = std::move(lanes);
  state_->window = window;
  state_->clock = &clock;
  state_->external_stop = std::move(stop);
  state_->prefetch_ns = prefetch_ns;
  state_->result.lanes.resize(state_->lanes.size());
}

ShadowVideo::~ShadowVideo() {
  if (!state_) return;
  state_->local_stop->request_stop();
  state_->ready.notify_all();
  for (auto& thread : state_->threads) {
    if (thread.joinable()) thread.join();
  }
}

void ShadowVideo::start() {
  if (!state_->threads.empty()) {
    throw std::logic_error("环视模拟已经启动");
  }
  for (std::size_t index = 0; index < state_->lanes.size(); ++index) {
    state_->threads.emplace_back([this, index] {
      auto& lane = state_->lanes[index];
      auto& lane_result = state_->result.lanes[index];
      lane_result.name = lane.name;
      const int fd = ::open(lane.payload_path.c_str(),
                            O_RDONLY | O_CLOEXEC | O_DIRECT);
      if (fd < 0) {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->result.ok = false;
        state_->result.error = "无法打开环视模拟缓存: " + lane.name;
        return;
      }
      bool open = true;
      const auto close_fd = [&] {
        if (!open) return;
        ::close(fd);
        open = false;
      };
      try {
        const auto should_stop = [&] {
          return state_->local_stop->stop_requested() ||
                 state_->external_stop->stop_requested();
        };
        const auto end_index = scheduled_end_index(lane, state_->window);
        std::deque<std::pair<std::uint64_t, VideoFrameBuffer>> resident;
        std::uint64_t next = 0;
        while (next < lane.timestamps_ns.size() &&
               lane.timestamps_ns[next] < state_->window.start_ns) {
          ++next;
        }
        const auto fill_until = [&](std::uint64_t origin_ns) {
          while (next < lane.timestamps_ns.size() && next < end_index &&
                 lane.timestamps_ns[next] < origin_ns + state_->prefetch_ns) {
            VideoFrameBuffer buffer(lane.frame_bytes);
            if (!read_frame(fd, next, buffer, should_stop)) return;
            resident.emplace_back(next, std::move(buffer));
            ++lane_result.frames_read;
            ++next;
          }
        };
        fill_until(state_->window.start_ns);
        StartAnchors anchors;
        {
          std::unique_lock<std::mutex> lock(state_->mutex);
          while (!state_->play_started && !should_stop()) {
            state_->ready.wait_for(lock, std::chrono::milliseconds(20));
          }
          if (!state_->play_started || should_stop()) {
            close_fd();
            return;
          }
          anchors = state_->anchors;
        }
        const auto slots = build_video_slots(
            lane.timestamps_ns, lane.source_fps, lane.output_fps,
            state_->window, anchors.play_start_ns);
        for (std::size_t slot_index = 0; slot_index < slots.size();
             ++slot_index) {
          if (should_stop()) break;
          const auto& slot = slots[slot_index];
          if (resident.empty() || resident.front().first != slot.source_index) {
            throw std::runtime_error("环视模拟窗口缺帧: " + lane.name);
          }
          const auto wait = joint_sensor_player::cancellation_aware_sleep_until(
              *state_->clock, slot.deadline_ns, state_->local_stop.get());
          if (wait == joint_sensor_player::WaitStatus::Cancelled ||
              should_stop()) {
            break;
          }
          if (wait == joint_sensor_player::WaitStatus::Error) {
            throw std::runtime_error("环视模拟等待期限失败: " + lane.name);
          }
          const auto actual = state_->clock->now_ns();
          const auto lateness =
              actual > slot.deadline_ns ? actual - slot.deadline_ns : 0;
          lane_result.max_lateness_ns =
              std::max(lane_result.max_lateness_ns, lateness);
          ++lane_result.outputs;
          const bool last_repeat =
              slot_index + 1 == slots.size() ||
              slots[slot_index + 1].source_index != slot.source_index;
          if (last_repeat) {
            resident.pop_front();
            fill_until(slot.slot_time_ns);
          }
        }
        close_fd();
      } catch (const std::exception& error) {
        close_fd();
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->result.ok = false;
        state_->result.error = error.what();
        state_->local_stop->request_stop();
      }
    });
  }
}

void ShadowVideo::notify_play_start(const StartAnchors& anchors) {
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->anchors = anchors;
    state_->play_started = true;
  }
  state_->ready.notify_all();
}

ShadowVideoResult ShadowVideo::wait() {
  for (auto& thread : state_->threads) {
    if (thread.joinable()) thread.join();
  }
  state_->joined = true;
  if (state_->result.error.empty()) state_->result.ok = true;
  for (const auto& lane : state_->result.lanes) {
    if (lane.h2c_writes != 0) {
      state_->result.ok = false;
      state_->result.error = "环视模拟写了H2C: " + lane.name;
    }
  }
  return state_->result;
}

std::vector<ShadowLaneInput> load_shadow_lanes(
    const std::filesystem::path& manifest_path) {
  std::ifstream input(manifest_path);
  if (!input) {
    throw std::runtime_error("无法打开环视模拟清单: " +
                             manifest_path.string());
  }
  const auto manifest = nlohmann::json::parse(input);
  if (manifest.value("width", 0) != 1920 ||
      manifest.value("height", 0) != 1200 ||
      manifest.value("pixel_format", "") != "uyvy422") {
    throw std::runtime_error("环视模拟必须是1920x1200 UYVY");
  }
  const auto root = manifest_path.parent_path();
  std::vector<ShadowLaneInput> lanes;
  for (const auto& [name, entry] : manifest.at("lanes").items()) {
    ShadowLaneInput lane;
    lane.name = name;
    lane.payload_path = root / entry.at("payload_file").get<std::string>();
    lane.frame_bytes = entry.at("frame_bytes").get<std::uint64_t>();
    lane.source_fps = entry.at("source_fps").get<unsigned>();
    lane.output_fps = entry.at("output_fps").get<unsigned>();
    const auto stamp_path =
        root / entry.at("timestamps_file").get<std::string>();
    std::ifstream stamps(stamp_path);
    if (!stamps) {
      throw std::runtime_error("无法打开环视时间戳: " + stamp_path.string());
    }
    std::uint64_t timestamp = 0;
    while (stamps >> timestamp) lane.timestamps_ns.push_back(timestamp);
    if (lane.timestamps_ns.size() != entry.at("frame_count").get<std::uint64_t>()) {
      throw std::runtime_error("环视时间戳数量与帧数不一致: " + name);
    }
    if (lane.frame_bytes != 1920ULL * 1200ULL * 2ULL) {
      throw std::runtime_error("环视帧大小不是1920x1200 UYVY: " + name);
    }
    lanes.push_back(std::move(lane));
  }
  if (lanes.size() != 4) {
    throw std::runtime_error("环视模拟必须是四路");
  }
  return lanes;
}

}  // namespace joint_sensor_orchestrator
