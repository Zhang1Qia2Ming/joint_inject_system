#pragma once

#include <cstdint>

namespace joint_sensor_orchestrator {

struct PtpSample {
  std::uint64_t monotonic_ns;
  std::uint64_t phc_ns;
};

inline constexpr std::uint64_t kLidarGridNs = 100'000'000ULL;
inline constexpr std::uint64_t kD10PeriodNs = 33'334'333ULL;
inline constexpr std::uint64_t kVideoLeadNs = 20'000'000ULL;

std::uint64_t mono_to_phc(std::uint64_t mono_ns, const PtpSample& sample);
std::uint64_t phc_to_mono(std::uint64_t phc_ns, const PtpSample& sample);

std::uint64_t shared_shift_ns(std::uint64_t first_front_deadline_mono,
                              const PtpSample& sample);

std::uint64_t nearest_d10_edge_phc(std::uint64_t content_phc_ns,
                                   std::uint64_t phase_mod_ns,
                                   std::uint64_t period_ns);

std::uint64_t video_write_deadline_mono(
    std::uint64_t content_deadline_mono, std::uint64_t shared_shift_ns,
    const PtpSample& sample, std::uint64_t phase_mod_ns,
    std::uint64_t period_ns, std::uint64_t lead_ns);

}  // namespace joint_sensor_orchestrator
