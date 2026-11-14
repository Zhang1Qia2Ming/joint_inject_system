#pragma once

// 把普通文件 mmap 之后 mlock，让页留在内存里。
// 联合播放里用来钉住大约 1.1 GiB 的点云。视频窗口不走这里。

#include <cstdint>
#include <filesystem>
#include <utility>
#include <vector>

namespace joint_sensor_orchestrator {

// 析构前一直握着这些映射。联合播放用它钉住点云文件。
class PinnedFiles {
 public:
  PinnedFiles() = default;
  PinnedFiles(const PinnedFiles&) = delete;
  PinnedFiles& operator=(const PinnedFiles&) = delete;
  PinnedFiles(PinnedFiles&& other) noexcept;
  PinnedFiles& operator=(PinnedFiles&& other) noexcept;
  ~PinnedFiles();

  std::uint64_t locked_bytes() const noexcept { return locked_bytes_; }
  std::size_t file_count() const noexcept { return file_count_; }

 private:
  friend PinnedFiles pin_regular_files(
      const std::filesystem::path& root, std::uint64_t max_lock_bytes);

  struct Mapping {
    void* address{};
    std::size_t length{};
  };

  void release() noexcept;

  std::vector<Mapping> mappings_;
  std::uint64_t locked_bytes_{};
  std::size_t file_count_{};
};

// 钉住 root 下面的普通文件，直到对象析构。超过 max_lock_bytes 就停。
PinnedFiles pin_regular_files(const std::filesystem::path& root,
                              std::uint64_t max_lock_bytes);

}  // namespace joint_sensor_orchestrator
