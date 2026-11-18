#include "joint_sensor_orchestrator/ptp_schedule.hpp"
#include "test_framework.hpp"

using joint_sensor_orchestrator::PtpSample;
using joint_sensor_orchestrator::kD10PeriodNs;
using joint_sensor_orchestrator::kLidarGridNs;
using joint_sensor_orchestrator::kVideoLeadNs;
using joint_sensor_orchestrator::mono_to_phc;
using joint_sensor_orchestrator::nearest_d10_edge_phc;
using joint_sensor_orchestrator::shared_shift_ns;
using joint_sensor_orchestrator::video_write_deadline_mono;

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
