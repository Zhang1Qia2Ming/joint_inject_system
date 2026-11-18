#pragma once

#include <cstdint>
#include <filesystem>

namespace joint_sensor_orchestrator {

struct PtpSample {
  std::uint64_t monotonic_ns;
  std::uint64_t phc_ns;
};

struct D10Phase {
  std::uint64_t phase_mod_ns;
  std::uint64_t period_ns;
  std::uint64_t lead_ns;
};

inline constexpr std::uint64_t kLidarGridNs = 100'000'000ULL;
inline constexpr std::uint64_t kD10PeriodNs = 33'334'333ULL;
inline constexpr std::uint64_t kVideoLeadNs = 20'000'000ULL;

struct FrontLidarAnchor {
  std::uint64_t window_start_ns{};
  std::uint64_t frame_bag_log_time_ns{};
  std::uint64_t packet_interval_ns{};
  std::int64_t time_offset_us{};
};

struct PtpPlaybackPlan {
  PtpSample sample{};
  D10Phase phase{};
  FrontLidarAnchor front{};
};

D10Phase load_d10_phase(const std::filesystem::path& path);

PtpSample read_ptp_sample(const std::filesystem::path& device);

std::uint64_t joint_shift_ns(std::uint64_t play_start_ns,
                             const PtpPlaybackPlan& plan);

// deadline 已是“没有后续帧”的哨兵时原样返回。相加溢出则抛 overflow_error。
std::uint64_t add_shift_ns(std::uint64_t deadline_ns, std::uint64_t shift_ns);

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
