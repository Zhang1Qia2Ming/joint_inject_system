#pragma once

#include <cstdint>
#include <filesystem>
#include <utility>
#include <vector>

namespace joint_sensor_orchestrator {

// Holds mmap+mlock ranges so tmpfs/disk pages stay resident for playback.
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

PinnedFiles pin_regular_files(const std::filesystem::path& root,
                              std::uint64_t max_lock_bytes);

}  // namespace joint_sensor_orchestrator
