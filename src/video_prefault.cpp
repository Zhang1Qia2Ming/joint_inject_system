// 10 秒整窗路径：把要播的 UYVY 拷进 /dev/shm。40 秒滑动窗口不调用它。
// 拷完可以选择丢掉窗口外的页。视频本身不 mlock。

#include "joint_sensor_orchestrator/video_prefault.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace joint_sensor_orchestrator {
namespace {

std::uint64_t checked_add(std::uint64_t left, std::uint64_t right,
                          const char* message) {
  if (left > std::numeric_limits<std::uint64_t>::max() - right) {
    throw std::overflow_error(message);
  }
  return left + right;
}

std::uint64_t page_size_or_throw() {
  const auto page = ::sysconf(_SC_PAGESIZE);
  if (page <= 0) {
    throw std::runtime_error("无法获取系统页大小");
  }
  return static_cast<std::uint64_t>(page);
}

std::uint64_t clamp_lock_budget(std::uint64_t requested) {
  rlimit limit{};
  if (::getrlimit(RLIMIT_MEMLOCK, &limit) != 0) {
    return requested;
  }
  if (limit.rlim_cur == RLIM_INFINITY) {
    return requested;
  }
  return std::min(requested, static_cast<std::uint64_t>(limit.rlim_cur));
}

void advise_dontneed(int fd, std::uint64_t offset, std::uint64_t length,
                     std::uint64_t& accounted) {
  if (length == 0) return;
  if (offset > static_cast<std::uint64_t>(std::numeric_limits<off_t>::max()) ||
      length > static_cast<std::uint64_t>(std::numeric_limits<off_t>::max())) {
    return;
  }
  if (::posix_fadvise(fd, static_cast<off_t>(offset),
                      static_cast<off_t>(length),
                      POSIX_FADV_DONTNEED) == 0) {
    accounted = checked_add(accounted, length, "DONTNEED计数溢出");
  }
}

std::uint64_t filesystem_available_bytes(
    const std::filesystem::path& path) {
  struct statvfs buffer;
  if (::statvfs(path.c_str(), &buffer) != 0) {
    throw std::runtime_error("无法查询暂存文件系统空间: " + path.string());
  }
  return static_cast<std::uint64_t>(buffer.f_bavail) *
         static_cast<std::uint64_t>(buffer.f_frsize);
}

}  // namespace

VideoPlaybackPin::VideoPlaybackPin(VideoPlaybackPin&& other) noexcept {
  *this = std::move(other);
}

VideoPlaybackPin& VideoPlaybackPin::operator=(
    VideoPlaybackPin&& other) noexcept {
  if (this == &other) return *this;
  release();
  mappings_ = std::move(other.mappings_);
  staged_payload_paths_ = std::move(other.staged_payload_paths_);
  staged_root_ = std::move(other.staged_root_);
  frames_prefaulted_ = other.frames_prefaulted_;
  outside_bytes_dontneed_ = other.outside_bytes_dontneed_;
  locked_bytes_ = other.locked_bytes_;
  staged_bytes_ = other.staged_bytes_;
  other.frames_prefaulted_ = 0;
  other.outside_bytes_dontneed_ = 0;
  other.locked_bytes_ = 0;
  other.staged_bytes_ = 0;
  return *this;
}

VideoPlaybackPin::~VideoPlaybackPin() { release(); }

void VideoPlaybackPin::release() noexcept {
  for (auto& mapping : mappings_) {
    if (mapping.address != nullptr && mapping.length != 0) {
      ::munlock(mapping.address, mapping.length);
      ::munmap(mapping.address, mapping.length);
    }
    mapping.address = nullptr;
    mapping.length = 0;
  }
  mappings_.clear();
  locked_bytes_ = 0;
  for (auto& path : staged_payload_paths_) {
    if (!path.empty()) {
      std::error_code ec;
      std::filesystem::remove(path, ec);
      path.clear();
    }
  }
  if (!staged_root_.empty()) {
    std::error_code ec;
    std::filesystem::remove_all(staged_root_, ec);
    staged_root_.clear();
  }
  staged_bytes_ = 0;
}

void VideoPlaybackPin::apply_staged_payload_paths(VideoCache& cache) const {
  for (std::size_t index = 0; index < cache.lanes.size(); ++index) {
    if (!staged_payload_paths_[index].empty()) {
      cache.lanes[index].payload_path = staged_payload_paths_[index];
    }
  }
}

VideoPlaybackPin prefault_and_pin_video_playback_window(
    const VideoCache& cache, SourceWindow window,
    std::optional<std::uint64_t> max_relative_ns,
    const VideoPrefaultOptions& options) {
  if (window.start_ns > window.end_ns) {
    throw std::invalid_argument("预热窗口起点晚于终点");
  }
  std::uint64_t end_ns = window.end_ns;
  if (max_relative_ns.has_value()) {
    end_ns = std::min(
        end_ns, checked_add(window.start_ns, *max_relative_ns,
                            "视频预热窗口溢出"));
  }

  VideoPlaybackPin pin;
  const auto page = page_size_or_throw();
  auto lock_budget = options.max_lock_bytes == 0
                         ? 0ULL
                         : clamp_lock_budget(options.max_lock_bytes);
  std::vector<std::byte> scratch;

  std::array<std::set<std::uint64_t>, 3> lane_indices;
  std::uint64_t staged_need = 0;
  for (std::size_t lane_index = 0; lane_index < cache.lanes.size();
       ++lane_index) {
    const auto& lane = cache.lanes[lane_index];
    if (lane.frame_bytes == 0 || lane.payload_path.empty()) {
      throw std::runtime_error("视频预热缺少有效缓存: " + lane.name);
    }
    const auto slots = build_video_slots(
        lane.timestamps_ns, lane.source_fps, lane.output_fps, window, 0);
    for (const auto& slot : slots) {
      if (slot.slot_time_ns > end_ns) continue;
      lane_indices[lane_index].insert(slot.source_index);
    }
    staged_need = checked_add(
        staged_need,
        static_cast<std::uint64_t>(lane_indices[lane_index].size()) *
            lane.frame_bytes,
        "视频shm暂存需求溢出");
  }

  if (options.stage_to_shm && staged_need > 0) {
    const auto stage_root =
        options.stage_dir.empty()
            ? std::filesystem::path("/dev/shm/joint_uyvy_stage")
            : options.stage_dir;
    std::filesystem::create_directories(stage_root);
    const auto available = filesystem_available_bytes(stage_root);
    const std::uint64_t headroom = 512ULL * 1024 * 1024;
    if (available < staged_need + headroom) {
      throw std::runtime_error(
          "shm视频窗口暂存空间不足: need=" + std::to_string(staged_need) +
          " avail=" + std::to_string(available));
    }
    std::ostringstream name;
    name << "run-" << ::getpid() << "-"
         << (cache.cache_key.empty() ? "cache" : cache.cache_key.substr(0, 12));
    pin.staged_root_ = stage_root / name.str();
    std::filesystem::create_directories(pin.staged_root_);
  }

  for (std::size_t lane_index = 0; lane_index < cache.lanes.size();
       ++lane_index) {
    const auto& lane = cache.lanes[lane_index];
    const auto& source_indices = lane_indices[lane_index];
    const int fd =
        ::open(lane.payload_path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
      throw std::runtime_error("无法打开视频缓存进行预热: " +
                               lane.payload_path.string());
    }
    const auto file_bytes =
        static_cast<std::uint64_t>(lane.frame_count) * lane.frame_bytes;
    if (source_indices.empty()) {
      if (options.drop_outside_pages) {
        advise_dontneed(fd, 0, file_bytes, pin.outside_bytes_dontneed_);
      }
      ::close(fd);
      continue;
    }

    int stage_fd = -1;
    std::filesystem::path staged_path;
    if (!pin.staged_root_.empty()) {
      staged_path = pin.staged_root_ / (lane.name + ".uyvy");
      stage_fd = ::open(staged_path.c_str(),
                        O_RDWR | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
      if (stage_fd < 0) {
        ::close(fd);
        throw std::runtime_error("无法创建shm视频暂存文件: " +
                                 staged_path.string());
      }
      if (::ftruncate(stage_fd, static_cast<off_t>(file_bytes)) != 0) {
        ::close(stage_fd);
        ::close(fd);
        throw std::runtime_error("shm视频暂存ftruncate失败: " +
                                 staged_path.string());
      }
    }

    if (scratch.size() != lane.frame_bytes) {
      scratch.assign(static_cast<std::size_t>(lane.frame_bytes), std::byte{});
    }

    std::vector<std::pair<std::uint64_t, std::uint64_t>> keep_ranges;
    for (const auto source_index : source_indices) {
      if (source_index >= lane.frame_count) {
        if (stage_fd >= 0) ::close(stage_fd);
        ::close(fd);
        throw std::runtime_error("视频预热源帧越界: " + lane.name);
      }
      const auto offset = source_index * lane.frame_bytes;
      std::ptrdiff_t count;
      do {
        count = ::pread(fd, scratch.data(), scratch.size(),
                        static_cast<off_t>(offset));
      } while (count < 0 && errno == EINTR);
      if (count < 0 ||
          static_cast<std::size_t>(count) != scratch.size()) {
        if (stage_fd >= 0) ::close(stage_fd);
        ::close(fd);
        throw std::runtime_error("视频预热读取失败: " + lane.name);
      }
      ++pin.frames_prefaulted_;

      if (stage_fd >= 0) {
        std::ptrdiff_t written;
        do {
          written = ::pwrite(stage_fd, scratch.data(), scratch.size(),
                             static_cast<off_t>(offset));
        } while (written < 0 && errno == EINTR);
        if (written < 0 ||
            static_cast<std::size_t>(written) != scratch.size()) {
          ::close(stage_fd);
          ::close(fd);
          throw std::runtime_error("shm视频暂存写入失败: " + lane.name);
        }
        pin.staged_bytes_ = checked_add(pin.staged_bytes_, lane.frame_bytes,
                                        "shm暂存计数溢出");
      }

      const auto keep_end = offset + lane.frame_bytes;
      if (!keep_ranges.empty() && offset <= keep_ranges.back().second) {
        keep_ranges.back().second =
            std::max(keep_ranges.back().second, keep_end);
      } else {
        keep_ranges.emplace_back(offset, keep_end);
      }
    }

    const int lock_fd = stage_fd >= 0 ? stage_fd : fd;
    for (const auto& range : keep_ranges) {
      if (lock_budget < page) break;
      const auto map_start = range.first - (range.first % page);
      const auto map_end = ((range.second + page - 1) / page) * page;
      const auto map_len = map_end - map_start;
      if (map_len == 0 || map_len > lock_budget ||
          map_len > static_cast<std::uint64_t>(
                        std::numeric_limits<std::size_t>::max())) {
        continue;
      }
      void* address =
          ::mmap(nullptr, static_cast<std::size_t>(map_len), PROT_READ,
                 MAP_SHARED | MAP_POPULATE, lock_fd,
                 static_cast<off_t>(map_start));
      if (address == MAP_FAILED) continue;
      if (::mlock(address, static_cast<std::size_t>(map_len)) == 0) {
        pin.mappings_.push_back(
            {address, static_cast<std::size_t>(map_len)});
        pin.locked_bytes_ =
            checked_add(pin.locked_bytes_, map_len, "mlock计数溢出");
        lock_budget -= map_len;
      } else {
        ::munmap(address, static_cast<std::size_t>(map_len));
      }
    }
    if (stage_fd >= 0) {
      ::fsync(stage_fd);
      ::close(stage_fd);
      pin.staged_payload_paths_[lane_index] = std::move(staged_path);
    }

    if (options.drop_outside_pages) {
      if (!pin.staged_payload_paths_[lane_index].empty()) {
        advise_dontneed(fd, 0, file_bytes, pin.outside_bytes_dontneed_);
      } else {
        std::uint64_t cursor = 0;
        for (const auto& range : keep_ranges) {
          if (range.first > cursor) {
            advise_dontneed(fd, cursor, range.first - cursor,
                            pin.outside_bytes_dontneed_);
          }
          cursor = std::max(cursor, range.second);
        }
        if (cursor < file_bytes) {
          advise_dontneed(fd, cursor, file_bytes - cursor,
                          pin.outside_bytes_dontneed_);
        }
      }
    }
    ::close(fd);
  }
  return pin;
}

VideoPlaybackPin prefault_and_pin_video_playback_window(
    const VideoCache& cache, SourceWindow window,
    std::optional<std::uint64_t> max_relative_ns,
    std::uint64_t max_lock_bytes, bool drop_outside_pages) {
  VideoPrefaultOptions options;
  options.max_lock_bytes = max_lock_bytes;
  options.drop_outside_pages = drop_outside_pages;
  return prefault_and_pin_video_playback_window(cache, window,
                                                max_relative_ns, options);
}

std::uint64_t prefault_video_playback_window(
    const VideoCache& cache, SourceWindow window,
    std::optional<std::uint64_t> max_relative_ns) {
  return prefault_and_pin_video_playback_window(cache, window,
                                                max_relative_ns)
      .frames_prefaulted();
}

}  // namespace joint_sensor_orchestrator
