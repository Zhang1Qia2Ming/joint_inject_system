#include "joint_sensor_orchestrator/file_pin.hpp"
#include "test_framework.hpp"

#include <unistd.h>

#include <fstream>

TEST_CASE(file_pin_locks_regular_files_under_budget) {
  const auto root = std::filesystem::temp_directory_path() /
                    ("file_pin_" + std::to_string(::getpid()));
  std::filesystem::create_directories(root);
  {
    std::ofstream out(root / "a.bin", std::ios::binary);
    std::string bytes(4096, 'A');
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  }
  auto pinned = joint_sensor_orchestrator::pin_regular_files(
      root, 1ULL * 1024 * 1024);
  CHECK(pinned.file_count() == 1);
  CHECK(pinned.locked_bytes() >= 4096);
  std::filesystem::remove_all(root);
}
