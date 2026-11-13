#include "joint_sensor_player/clock_health.hpp"
#include "joint_sensor_player/config.hpp"
#include "joint_sensor_player/mcap.hpp"
#include "joint_sensor_player/packet_time_rewriter.hpp"
#include "joint_sensor_player/player.hpp"
#include "joint_sensor_player/ptc_server.hpp"
#include "joint_sensor_player/report.hpp"
#include "joint_sensor_player/timeline.hpp"
#include "joint_sensor_player/udp_sender.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>

#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

volatile std::sig_atomic_t g_signal_stop = 0;

void handle_stop_signal(int) { g_signal_stop = 1; }

struct Arguments {
  std::filesystem::path config;
  bool dry_run{};
};

void usage(std::ostream& output) {
  output << "用法: hil_lidar_emulator --config FILE [--dry-run]\n"
            "  --dry-run  扫描MCAP、校验八路协议和连续时间，但不发送UDP\n";
}

Arguments parse_arguments(int argc, char** argv) {
  Arguments result;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      usage(std::cout);
      std::exit(0);
    }
    if (argument == "--dry-run") {
      result.dry_run = true;
      continue;
    }
    if (argument == "--config" && index + 1 < argc) {
      result.config = argv[++index];
      continue;
    }
    throw std::runtime_error("未知或不完整参数: " + argument);
  }
  if (result.config.empty()) throw std::runtime_error("缺少--config FILE");
  return result;
}

std::string read_text(const std::filesystem::path& path) {
  std::ifstream input(path);
  std::string value;
  std::getline(input, value);
  return value;
}

std::vector<std::uint8_t> read_binary(
    const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("无法读取correction: " + path.string());
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

std::set<std::string> interface_addresses(const std::string& interface) {
  ifaddrs* list = nullptr;
  if (::getifaddrs(&list) != 0) throw std::runtime_error("getifaddrs失败");
  std::set<std::string> addresses;
  for (auto* current = list; current != nullptr; current = current->ifa_next) {
    if (current->ifa_addr == nullptr ||
        current->ifa_addr->sa_family != AF_INET ||
        interface != current->ifa_name) {
      continue;
    }
    char text[INET_ADDRSTRLEN]{};
    const auto* address =
        &reinterpret_cast<sockaddr_in*>(current->ifa_addr)->sin_addr;
    if (::inet_ntop(AF_INET, address, text, sizeof(text)) != nullptr) {
      addresses.insert(text);
    }
  }
  ::freeifaddrs(list);
  return addresses;
}

bool address_exists_on_other_interface(const std::string& address,
                                       const std::string& interface) {
  ifaddrs* list = nullptr;
  if (::getifaddrs(&list) != 0) throw std::runtime_error("getifaddrs失败");
  bool found = false;
  for (auto* current = list; current != nullptr; current = current->ifa_next) {
    if (current->ifa_addr == nullptr ||
        current->ifa_addr->sa_family != AF_INET ||
        interface == current->ifa_name) {
      continue;
    }
    char text[INET_ADDRSTRLEN]{};
    const auto* value =
        &reinterpret_cast<sockaddr_in*>(current->ifa_addr)->sin_addr;
    if (::inet_ntop(AF_INET, value, text, sizeof(text)) != nullptr &&
        address == text) {
      found = true;
      break;
    }
  }
  ::freeifaddrs(list);
  return found;
}

void network_preflight(const joint_sensor_player::PlayerConfig& config) {
  const auto base = std::filesystem::path("/sys/class/net") / config.interface;
  if (!std::filesystem::exists(base)) {
    throw std::runtime_error("网口不存在: " + config.interface);
  }
  if (read_text(base / "carrier") != "1") {
    throw std::runtime_error(config.interface + "没有物理链路");
  }
  const int speed = std::stoi(read_text(base / "speed"));
  if (speed < 1000 || read_text(base / "duplex") != "full") {
    throw std::runtime_error(config.interface +
                             "必须达到至少1000Mbps Full Duplex");
  }
  const auto addresses = interface_addresses(config.interface);
  for (const auto& channel : config.channels) {
    if (!channel.enabled) continue;
    if (!addresses.contains(channel.source_ip)) {
      throw std::runtime_error("网口缺少雷达源IP: " + channel.source_ip +
                               "，先运行prepare_network.sh");
    }
    if (address_exists_on_other_interface(channel.source_ip,
                                          config.interface)) {
      throw std::runtime_error("雷达源IP同时存在于其他网口: " +
                               channel.source_ip);
    }
  }
}

std::filesystem::path create_run_directory(
    const std::filesystem::path& report_root) {
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&time, &local);
  const auto micros =
      std::chrono::duration_cast<std::chrono::microseconds>(
          now.time_since_epoch())
          .count() %
      1000000;
  std::ostringstream base;
  base << "emulator-" << std::put_time(&local, "%Y%m%d-%H%M%S") << '-'
       << std::setw(6) << std::setfill('0') << micros;
  std::filesystem::create_directories(report_root);
  for (unsigned suffix = 0; suffix < 1000; ++suffix) {
    const auto name =
        suffix == 0 ? base.str() : base.str() + "-" + std::to_string(suffix);
    const auto path = report_root / name;
    if (std::filesystem::create_directory(path)) return path;
  }
  throw std::runtime_error("无法创建唯一报告目录");
}

std::uint64_t cycle_period_ns(
    const joint_sensor_player::McapCatalog& catalog,
    const joint_sensor_player::PlayerConfig& config,
    const joint_sensor_player::PlaybackWindow& window) {
  const auto frame_period =
      joint_sensor_player::nominal_frame_period_ns(catalog, config.channels);
  const auto duration = window.source_end_ns - window.source_start_ns;
  if (duration > std::numeric_limits<std::uint64_t>::max() - frame_period) {
    throw std::runtime_error("循环周期溢出");
  }
  return duration + frame_period;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto arguments = parse_arguments(argc, argv);
    const auto config = joint_sensor_player::load_config(arguments.config);
    std::cout << "扫描MCAP目录: " << config.data_dir << '\n';
    auto catalog =
        joint_sensor_player::scan_mcap_directory(config.data_dir,
                                                 config.channels);
    const auto window = joint_sensor_player::build_playback_window(
        catalog, config.channels, config.allow_partial_start);
    const auto period_ns = cycle_period_ns(catalog, config, window);
    network_preflight(config);

    const auto clock_health = joint_sensor_player::inspect_system_clock(
        config.require_synchronized_clock);
    if (clock_health.realtime_us >
        std::numeric_limits<std::uint64_t>::max() -
            static_cast<std::uint64_t>(config.start_delay_ms) * 1000ULL) {
      throw std::runtime_error("UTC锚点溢出");
    }
    const auto anchor_utc_us =
        clock_health.realtime_us +
        static_cast<std::uint64_t>(config.start_delay_ms) * 1000ULL;
    const auto rewriter = joint_sensor_player::LidarPacketRewriter::build(
        catalog, config.channels, window, period_ns, anchor_utc_us);

    const auto run_dir = create_run_directory(config.report_dir);
    std::filesystem::copy_file(
        arguments.config, run_dir / "config_snapshot.yaml",
        std::filesystem::copy_options::overwrite_existing);
    joint_sensor_player::AsyncReport report(run_dir);
    for (const auto& annotation : catalog.annotations) {
      report.add_anomaly(annotation.kind,
                         annotation.topic + ": " + annotation.detail);
    }
    report.log("mode=hil_lidar_emulator");
    report.log("mcap_files=" + std::to_string(catalog.files.size()));
    report.log("clock_synchronized=" +
               std::string(clock_health.synchronized ? "true" : "false"));
    report.log("clock_degraded=" +
               std::string(clock_health.degraded ? "true" : "false"));
    report.log("anchor_utc_us=" + std::to_string(anchor_utc_us));
    report.log("source_origin_us=" +
               std::to_string(rewriter.source_origin_us()));
    report.log("cycle_period_us=" +
               std::to_string(rewriter.cycle_period_us()));
    for (const auto& channel : config.channels) {
      if (!channel.enabled) continue;
      const auto& range = rewriter.channel_ranges().at(channel.topic);
      const auto target_first =
          anchor_utc_us + range.first_us - rewriter.source_origin_us();
      const auto target_last =
          anchor_utc_us + range.last_us - rewriter.source_origin_us();
      report.log("channel_time slot=" + std::to_string(channel.slot) +
                 " topic=" + channel.topic +
                 " source_first_us=" + std::to_string(range.first_us) +
                 " source_last_us=" + std::to_string(range.last_us) +
                 " target_cycle0_first_us=" +
                 std::to_string(target_first) +
                 " target_cycle0_last_us=" + std::to_string(target_last));
    }
    std::cout << "八路协议预检通过，UTC锚点=" << anchor_utc_us
              << "，循环周期=" << rewriter.cycle_period_us() / 1000
              << " ms，时钟同步="
              << (clock_health.synchronized ? "yes" : "no") << '\n';

    joint_sensor_player::ReportSummary summary;
    summary.files_scanned = catalog.files.size();
    summary.duplicate_messages = catalog.duplicate_messages;
    if (arguments.dry_run) {
      summary.ok = true;
      summary.stop_reason = "dry_run";
      report.log("time_rollback_count=0");
      report.finish(summary);
      std::cout << "DRY_RUN_OK，报告: " << run_dir << '\n';
      return 0;
    }

    std::unique_ptr<joint_sensor_player::PtcCalibrationService> ptc_service;
    if (config.ptc_enabled) {
      std::vector<joint_sensor_player::PtcEndpoint> endpoints;
      for (const auto& channel : config.channels) {
        if (!channel.enabled) continue;
        endpoints.push_back({channel.source_ip, config.ptc_port,
                             read_binary(channel.correction_file)});
      }
      ptc_service =
          std::make_unique<joint_sensor_player::PtcCalibrationService>(
              std::move(endpoints));
      ptc_service->start();
      report.log("ptc_calibration_service=enabled");
      std::cout << "PTC标定服务已启动: 八路TCP " << config.ptc_port << '\n';
    }

    joint_sensor_player::UdpDatagramSink sink(config);
    joint_sensor_player::MonotonicClock clock;
    joint_sensor_player::CancellationToken cancellation(&g_signal_stop);
    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);
    std::cout << "将在" << config.start_delay_ms
              << " ms后按严格1倍速模拟真雷达，Ctrl-C停止\n";
    const auto result = joint_sensor_player::play_catalog(
        config, catalog, clock, sink, &report, &cancellation, &rewriter);
    report.log("packets_sent=" + std::to_string(result.packets_sent));
    report.log("frames_sent=" + std::to_string(result.frames_sent));
    report.log("loops_completed=" + std::to_string(result.loops_completed));
    report.log("time_rollback_count=0");
    summary.ok = result.ok;
    summary.frames_sent = result.frames_sent;
    summary.packets_sent = result.packets_sent;
    summary.max_lateness_ns = result.max_lateness_ns;
    summary.loops_completed = result.loops_completed;
    summary.stop_reason = result.stop_reason;
    summary.error = result.error;
    report.finish(summary);
    if (!result.ok) {
      std::cerr << "仿真失败: " << result.error << '\n';
      return 3;
    }
    std::cout << "仿真结束: loops=" << result.loops_completed
              << " reason=" << result.stop_reason
              << " frames=" << result.frames_sent
              << " packets=" << result.packets_sent
              << " max_lateness_us=" << result.max_lateness_ns / 1000
              << " report=" << run_dir << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "错误: " << error.what() << '\n';
    usage(std::cerr);
    return 2;
  }
}
