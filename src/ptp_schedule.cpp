#include "joint_sensor_orchestrator/ptp_schedule.hpp"

#include <limits>
#include <stdexcept>

namespace joint_sensor_orchestrator {

std::uint64_t mono_to_phc(const std::uint64_t mono_ns,
                          const PtpSample& sample) {
  return sample.phc_ns + (mono_ns - sample.monotonic_ns);
}

std::uint64_t phc_to_mono(const std::uint64_t phc_ns,
                          const PtpSample& sample) {
  if (phc_ns < sample.phc_ns) {
    const auto offset_ns = sample.phc_ns - phc_ns;
    if (sample.monotonic_ns < offset_ns) {
      throw std::overflow_error("converted monotonic time is earlier than zero");
    }
    return sample.monotonic_ns - offset_ns;
  }

  const auto offset_ns = phc_ns - sample.phc_ns;
  if (sample.monotonic_ns >
      std::numeric_limits<std::uint64_t>::max() - offset_ns) {
    throw std::overflow_error("converted monotonic time overflows");
  }
  return sample.monotonic_ns + offset_ns;
}

std::uint64_t shared_shift_ns(
    const std::uint64_t first_front_deadline_mono, const PtpSample& sample) {
  const auto deadline_phc = mono_to_phc(first_front_deadline_mono, sample);
  const auto remainder = deadline_phc % kLidarGridNs;
  return remainder == 0 ? 0 : kLidarGridNs - remainder;
}

std::uint64_t nearest_d10_edge_phc(const std::uint64_t content_phc_ns,
                                   const std::uint64_t phase_mod_ns,
                                   const std::uint64_t period_ns) {
  if (phase_mod_ns >= period_ns) {
    throw std::invalid_argument("D10 phase must be smaller than its period");
  }

  if (content_phc_ns <= phase_mod_ns) {
    return phase_mod_ns;
  }

  const auto periods_after_phase =
      (content_phc_ns - phase_mod_ns) / period_ns;
  const auto earlier_edge =
      phase_mod_ns + periods_after_phase * period_ns;
  const auto distance_to_earlier = content_phc_ns - earlier_edge;

  if (earlier_edge > std::numeric_limits<std::uint64_t>::max() - period_ns) {
    return earlier_edge;
  }

  const auto later_edge = earlier_edge + period_ns;
  const auto distance_to_later = later_edge - content_phc_ns;
  return distance_to_earlier <= distance_to_later ? earlier_edge : later_edge;
}

std::uint64_t video_write_deadline_mono(
    const std::uint64_t content_deadline_mono,
    const std::uint64_t shared_shift_ns, const PtpSample& sample,
    const std::uint64_t phase_mod_ns, const std::uint64_t period_ns,
    const std::uint64_t lead_ns) {
  const auto shifted_content_mono =
      content_deadline_mono + shared_shift_ns;
  const auto content_phc = mono_to_phc(shifted_content_mono, sample);
  const auto edge_phc =
      nearest_d10_edge_phc(content_phc, phase_mod_ns, period_ns);
  const auto edge_mono = phc_to_mono(edge_phc, sample);

  if (edge_mono < lead_ns) {
    throw std::overflow_error("video write deadline is earlier than zero");
  }
  return edge_mono - lead_ns;
}

}  // namespace joint_sensor_orchestrator
