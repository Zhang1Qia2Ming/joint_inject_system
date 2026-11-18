#include "joint_sensor_orchestrator/ptp_schedule.hpp"
#include "test_framework.hpp"

#include <atomic>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

using joint_sensor_orchestrator::D10Phase;
using joint_sensor_orchestrator::PtpSample;
using joint_sensor_orchestrator::kD10PeriodNs;
using joint_sensor_orchestrator::kLidarGridNs;
using joint_sensor_orchestrator::kVideoLeadNs;
using joint_sensor_orchestrator::load_d10_phase;
using joint_sensor_orchestrator::mono_to_phc;
using joint_sensor_orchestrator::nearest_d10_edge_phc;
using joint_sensor_orchestrator::shared_shift_ns;
using joint_sensor_orchestrator::video_write_deadline_mono;

namespace {

class TemporaryPhaseFile {
 public:
  explicit TemporaryPhaseFile(const std::string& contents) {
    static std::atomic<unsigned int> sequence{0};
    path_ = std::filesystem::temp_directory_path() /
            ("d10_phase_" + std::to_string(sequence++) + ".txt");
    std::ofstream output(path_);
    output << contents;
  }

  ~TemporaryPhaseFile() { std::filesystem::remove(path_); }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

const std::string kValidPhaseFile =
    "phase_mod_ns=1000000\n"
    "period_ns=33334333\n"
    "lead_ns=20000000\n"
    "uncertainty=millisecond\n";

std::string load_error(const std::string& contents) {
  const TemporaryPhaseFile file(contents);
  try {
    (void)load_d10_phase(file.path());
  } catch (const std::runtime_error& error) {
    return error.what();
  }
  return {};
}

}  // namespace

TEST_CASE(shift_moves_front_lidar_onto_next_100ms) {
  const PtpSample sample{1'000'000'000ULL, 5'000'000'000ULL};
  const auto deadline = sample.monotonic_ns + 50'000'000ULL;
  const auto shift = shared_shift_ns(deadline, sample);
  CHECK(shift == 50'000'000ULL);
  CHECK(mono_to_phc(deadline + shift, sample) % kLidarGridNs == 0);
}

TEST_CASE(already_aligned_shift_is_zero) {
  const PtpSample sample{1'000'000'000ULL, 5'000'000'000ULL};
  CHECK(shared_shift_ns(sample.monotonic_ns, sample) == 0);
}

TEST_CASE(nearest_edge_picks_closer_side) {
  const auto period = kD10PeriodNs;
  const auto phase = 1'000'000ULL;
  CHECK(nearest_d10_edge_phc(phase + 100ULL, phase, period) == phase);
  CHECK(nearest_d10_edge_phc(phase + period - 100ULL, phase, period) ==
        phase + period);
}

TEST_CASE(video_write_is_20ms_before_predicted_edge) {
  const PtpSample sample{0, 0};
  const auto content = kD10PeriodNs;
  const auto write = video_write_deadline_mono(
      content, 0, sample, 0, kD10PeriodNs, kVideoLeadNs);
  CHECK(write == content - kVideoLeadNs);
}

TEST_CASE(video_write_rejects_deadline_before_zero) {
  const PtpSample sample{100, 150};
  CHECK_THROWS(video_write_deadline_mono(100, 0, sample, 0, 1'000, 20));
}

TEST_CASE(d10_phase_loads_millisecond_file) {
  const TemporaryPhaseFile file(kValidPhaseFile);
  const D10Phase phase = load_d10_phase(file.path());
  CHECK(phase.phase_mod_ns == 1'000'000ULL);
  CHECK(phase.period_ns == kD10PeriodNs);
  CHECK(phase.lead_ns == kVideoLeadNs);
}

TEST_CASE(d10_phase_rejects_microsecond_uncertainty) {
  auto contents = kValidPhaseFile;
  contents.replace(contents.find("millisecond"), std::string("millisecond").size(),
                   "microsecond");
  CHECK(load_error(contents).find("毫秒级") != std::string::npos);
}

TEST_CASE(d10_phase_rejects_each_missing_line) {
  const std::string lines[] = {
      "phase_mod_ns=1000000\n",
      "period_ns=33334333\n",
      "lead_ns=20000000\n",
      "uncertainty=millisecond\n",
  };
  for (const auto& omitted : lines) {
    auto contents = kValidPhaseFile;
    contents.erase(contents.find(omitted), omitted.size());
    CHECK(!load_error(contents).empty());
  }
}

TEST_CASE(d10_phase_rejects_phase_outside_period) {
  auto contents = kValidPhaseFile;
  contents.replace(contents.find("phase_mod_ns=1000000"),
                   std::string("phase_mod_ns=1000000").size(),
                   "phase_mod_ns=33334333");
  CHECK(!load_error(contents).empty());
}

TEST_CASE(d10_phase_rejects_negative_period) {
  auto contents = kValidPhaseFile;
  contents.replace(contents.find("period_ns=33334333"),
                   std::string("period_ns=33334333").size(), "period_ns=-1");
  CHECK(!load_error(contents).empty());
}

TEST_CASE(d10_phase_rejects_unexpected_lead) {
  auto contents = kValidPhaseFile;
  contents.replace(contents.find("lead_ns=20000000"),
                   std::string("lead_ns=20000000").size(),
                   "lead_ns=10000000");
  CHECK(!load_error(contents).empty());
}
