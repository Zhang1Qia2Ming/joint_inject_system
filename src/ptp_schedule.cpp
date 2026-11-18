#include "joint_sensor_orchestrator/ptp_schedule.hpp"

#include "joint_sensor_player/timeline.hpp"

#include <fcntl.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

namespace joint_sensor_orchestrator {

namespace {

std::uint64_t parse_ns(const std::string& value, const std::string& key) {
  if (value.empty() || value.front() == '-') {
    throw std::runtime_error("相位文件中的 " + key + " 不是有效整数");
  }
  for (const char digit : value) {
    if (digit < '0' || digit > '9') {
      throw std::runtime_error("相位文件中的 " + key + " 不是有效整数");
    }
  }

  std::size_t consumed = 0;
  try {
    const auto parsed = std::stoull(value, &consumed);
    if (consumed != value.size()) {
      throw std::runtime_error("invalid numeric value");
    }
    return parsed;
  } catch (const std::exception&) {
    throw std::runtime_error("相位文件中的 " + key + " 不是有效整数");
  }
}

}  // namespace

D10Phase load_d10_phase(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("无法打开相位文件: " + path.string());
  }

  D10Phase phase{};
  bool has_phase = false;
  bool has_period = false;
  bool has_lead = false;
  bool has_uncertainty = false;

  std::string line;
  while (std::getline(input, line)) {
    const auto separator = line.find('=');
    if (separator == std::string::npos) {
      continue;
    }
    const auto key = line.substr(0, separator);
    const auto value = line.substr(separator + 1);
    if (key == "phase_mod_ns") {
      phase.phase_mod_ns = parse_ns(value, key);
      has_phase = true;
    } else if (key == "period_ns") {
      phase.period_ns = parse_ns(value, key);
      has_period = true;
    } else if (key == "lead_ns") {
      phase.lead_ns = parse_ns(value, key);
      has_lead = true;
    } else if (key == "uncertainty") {
      if (value != "millisecond") {
        throw std::runtime_error("相位文件只接受毫秒级不确定性");
      }
      has_uncertainty = true;
    }
  }

  if (!has_phase || !has_period || !has_lead || !has_uncertainty) {
    throw std::runtime_error("相位文件缺少必需字段");
  }
  if (phase.phase_mod_ns >= phase.period_ns) {
    throw std::runtime_error("phase_mod_ns 必须小于 period_ns");
  }
  if (phase.lead_ns != kVideoLeadNs) {
    throw std::runtime_error("lead_ns 必须等于视频提前量");
  }
  return phase;
}

namespace {

std::uint64_t timespec_to_ns(const timespec& value) {
  if (value.tv_sec < 0 || value.tv_nsec < 0 ||
      value.tv_nsec >= 1'000'000'000L) {
    throw std::runtime_error("PTP时钟读数无效");
  }
  const auto seconds = static_cast<std::uint64_t>(value.tv_sec);
  if (seconds >
      std::numeric_limits<std::uint64_t>::max() / 1'000'000'000ULL) {
    throw std::runtime_error("PTP时钟读数溢出");
  }
  return seconds * 1'000'000'000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
}

}  // namespace

PtpSample read_ptp_sample(const std::filesystem::path& device) {
  const int fd = ::open(device.c_str(), O_RDONLY);
  if (fd < 0) {
    throw std::runtime_error("无法打开PTP设备: " + device.string());
  }
  // FD_TO_CLOCKID(fd) == (~(clockid_t)fd << 3) | 3。用无符号移位避免负值左移。
  const auto fd_bits = static_cast<std::uint32_t>(fd);
  const auto clock_id =
      static_cast<clockid_t>((~fd_bits << 3) | 3u);
  timespec phc_time{};
  timespec mono_time{};
  const int phc_rc = ::clock_gettime(clock_id, &phc_time);
  const int mono_rc = ::clock_gettime(CLOCK_MONOTONIC, &mono_time);
  ::close(fd);
  if (phc_rc != 0 || mono_rc != 0) {
    throw std::runtime_error("无法读取PTP时钟: " + device.string());
  }
  return {timespec_to_ns(mono_time), timespec_to_ns(phc_time)};
}

std::uint64_t joint_shift_ns(const std::uint64_t play_start_ns,
                             const PtpPlaybackPlan& plan) {
  const auto deadline = joint_sensor_player::packet_deadline_ns(
      play_start_ns, plan.front.window_start_ns,
      plan.front.frame_bag_log_time_ns, 0, plan.front.packet_interval_ns,
      plan.front.time_offset_us);
  return shared_shift_ns(deadline, plan.sample);
}

std::uint64_t add_shift_ns(const std::uint64_t deadline_ns,
                           const std::uint64_t shift_ns) {
  if (deadline_ns == std::numeric_limits<std::uint64_t>::max()) {
    return deadline_ns;
  }
  if (shift_ns >
      std::numeric_limits<std::uint64_t>::max() - deadline_ns) {
    throw std::overflow_error("PTP位移后的发送期限溢出");
  }
  return deadline_ns + shift_ns;
}

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
