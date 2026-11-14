// mmap 后 mlock。联合播放用它把点云留在内存里，视频帧不从这里钉。

#include "joint_sensor_orchestrator/file_pin.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>

namespace joint_sensor_orchestrator {
namespace {

std::uint64_t page_size_or_throw() {
  const auto page = ::sysconf(_SC_PAGESIZE);
  if (page <= 0) {
    throw std::runtime_error("无法获取系统页大小");
  }
  return static_cast<std::uint64_t>(page);
}

std::uint64_t clamp_lock_budget(std::uint64_t requested) {
  struct rlimit limit {};
  if (::getrlimit(RLIMIT_MEMLOCK, &limit) != 0) {
    return requested;
  }
  if (limit.rlim_cur == RLIM_INFINITY) {
    return requested;
  }
  return std::min(requested, static_cast<std::uint64_t>(limit.rlim_cur));
}

}  // namespace

PinnedFiles::PinnedFiles(PinnedFiles&& other) noexcept {
  *this = std::move(other);
}

PinnedFiles& PinnedFiles::operator=(PinnedFiles&& other) noexcept {
  if (this != &other) {
    release();
    mappings_ = std::move(other.mappings_);
    locked_bytes_ = other.locked_bytes_;
    file_count_ = other.file_count_;
    other.locked_bytes_ = 0;
    other.file_count_ = 0;
  }
  return *this;
}

PinnedFiles::~PinnedFiles() { release(); }

void PinnedFiles::release() noexcept {
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
  file_count_ = 0;
}

PinnedFiles pin_regular_files(const std::filesystem::path& root,
                              std::uint64_t max_lock_bytes) {
  if (max_lock_bytes == 0) {
    return {};
  }
  if (!std::filesystem::exists(root)) {
    throw std::runtime_error("锁定目录不存在: " + root.string());
  }
  PinnedFiles pin;
  auto budget = clamp_lock_budget(max_lock_bytes);
  const auto page = page_size_or_throw();

  const auto try_pin_file = [&](const std::filesystem::path& path) {
    if (budget < page) return;
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
      throw std::runtime_error("无法打开文件进行mlock: " + path.string());
    }
    struct stat info {};
    if (::fstat(fd, &info) != 0 || info.st_size < 0) {
      ::close(fd);
      throw std::runtime_error("无法stat文件进行mlock: " + path.string());
    }
    const auto file_bytes = static_cast<std::uint64_t>(info.st_size);
    if (file_bytes == 0) {
      ::close(fd);
      return;
    }
    const auto map_len = ((file_bytes + page - 1) / page) * page;
    if (map_len > budget ||
        map_len > static_cast<std::uint64_t>(
                      std::numeric_limits<std::size_t>::max())) {
      ::close(fd);
      return;
    }
    // MAP_SHARED so mlock pins the file page-cache pages that pread/read
    // will hit (MAP_PRIVATE would lock a useless private COW copy).
    void* address =
        ::mmap(nullptr, static_cast<std::size_t>(map_len), PROT_READ,
               MAP_SHARED | MAP_POPULATE, fd, 0);
    ::close(fd);
    if (address == MAP_FAILED) {
      throw std::runtime_error("mmap失败无法mlock: " + path.string());
    }
    if (::mlock(address, static_cast<std::size_t>(map_len)) != 0) {
      ::munmap(address, static_cast<std::size_t>(map_len));
      return;
    }
    pin.mappings_.push_back({address, static_cast<std::size_t>(map_len)});
    pin.locked_bytes_ += map_len;
    budget -= map_len;
    ++pin.file_count_;
  };

  if (std::filesystem::is_regular_file(root)) {
    try_pin_file(root);
    return pin;
  }
  for (const auto& entry :
       std::filesystem::recursive_directory_iterator(root)) {
    if (!entry.is_regular_file()) continue;
    try_pin_file(entry.path());
  }
  return pin;
}

}  // namespace joint_sensor_orchestrator
