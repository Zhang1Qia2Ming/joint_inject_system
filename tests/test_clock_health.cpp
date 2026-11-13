#include "joint_sensor_player/clock_health.hpp"
#include "joint_sensor_player/config.hpp"
#include "test_framework.hpp"

#include <filesystem>
#include <fstream>

TEST_CASE(clock_config_defaults_strict_and_supports_explicit_degrade) {
  const auto path =
      std::filesystem::temp_directory_path() / "hil_clock_config.yaml";
  {
    std::ofstream out(path);
    out << "clock:\n"
           "  require_synchronized: false\n";
  }
  const auto config = joint_sensor_player::load_config(path);
  std::filesystem::remove(path);
  CHECK(!config.require_synchronized_clock);
}

TEST_CASE(clock_health_rejects_bad_time_and_required_unsynchronized_clock) {
  CHECK_THROWS(joint_sensor_player::validate_clock_health(
      1000000ULL, true, true));
  CHECK_THROWS(joint_sensor_player::validate_clock_health(
      1792549044000000ULL, false, true));
}

TEST_CASE(clock_health_marks_explicit_unsynchronized_mode_degraded) {
  const auto status = joint_sensor_player::validate_clock_health(
      1792549044000000ULL, false, false);
  CHECK(status.realtime_us == 1792549044000000ULL);
  CHECK(!status.synchronized);
  CHECK(status.degraded);
}
