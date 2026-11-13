#include "joint_sensor_player/config.hpp"
#include "joint_sensor_player/mcap.hpp"
#include "joint_sensor_player/packet_sequence.hpp"
#include "joint_sensor_player/player.hpp"
#include "joint_sensor_player/report.hpp"
#include "joint_sensor_player/timeline.hpp"
#include "joint_sensor_player/udp_sender.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>

#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
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
  output << "用法: joint_sensor_player --config FILE [--dry-run]\n"
            "  --dry-run  扫描MCAP并完成严格启动检查，但不发送UDP\n";
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

std::set<std::string> interface_addresses(const std::string& interface) {
  ifaddrs* list = nullptr;
  if (::getifaddrs(&list) != 0) throw std::runtime_error("getifaddrs失败");
  std::set<std::string> addresses;
  for (auto* current = list; current != nullptr; current = current->ifa_next) {
    if (current->ifa_addr == nullptr || current->ifa_addr->sa_family != AF_INET ||
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
    if (channel.enabled && !addresses.contains(channel.source_ip)) {
      throw std::runtime_error("网口缺少雷达源IP: " + channel.source_ip +
                               "，先运行prepare_network.sh");
    }
    if (channel.enabled &&
        address_exists_on_other_interface(channel.source_ip,
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
  base << std::put_time(&local, "%Y%m%d-%H%M%S") << '-' << std::setw(6)
       << std::setfill('0') << micros;
  std::filesystem::create_directories(report_root);
  for (unsigned suffix = 0; suffix < 1000; ++suffix) {
    const auto name =
        suffix == 0 ? base.str() : base.str() + "-" + std::to_string(suffix);
    const auto path = report_root / name;
    if (std::filesystem::create_directory(path)) return path;
  }
  throw std::runtime_error("无法创建唯一报告目录");
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto arguments = parse_arguments(argc, argv);
    const auto config = joint_sensor_player::load_config(arguments.config);
    std::cout << "扫描MCAP目录: " << config.data_dir << '\n';
    auto catalog =
        joint_sensor_player::scan_mcap_directory(config.data_dir, config.channels);
    const auto window = joint_sensor_player::build_playback_window(
        catalog, config.channels, config.allow_partial_start);
    if (config.loop_count == 0 || config.loop_count > 1) {
      const auto period =
          joint_sensor_player::nominal_frame_period_ns(catalog, config.channels);
      if (arguments.dry_run &&
          config.sequence_mode ==
              joint_sensor_player::SequenceMode::Continuous) {
        const auto plans = joint_sensor_player::build_sequence_plans(
            catalog, config.channels, config.sequence_mode,
            window.source_start_ns, window.source_end_ns);
        if (plans.size() != window.enabled_topics) {
          throw std::runtime_error("循环序号计划没有覆盖全部启用topic");
        }
      }
      std::cout << "循环模式: "
                << (config.loop_count == 0
                        ? std::string("无限")
                        : std::to_string(config.loop_count) + "轮")
                << "，序号="
                << (config.sequence_mode ==
                            joint_sensor_player::SequenceMode::Continuous
                        ? "continuous"
                        : "preserve")
                << "，帧周期=" << period / 1000000 << " ms\n";
    } else {
      std::cout << "循环模式: 单次播放\n";
    }
    std::cout << "MCAP文件: " << catalog.files.size()
              << "，共同播放区间: "
              << (window.source_end_ns - window.source_start_ns) / 1000000
              << " ms\n";
    network_preflight(config);

    const auto run_dir = create_run_directory(config.report_dir);
    std::filesystem::copy_file(
        arguments.config, run_dir / "config_snapshot.yaml",
        std::filesystem::copy_options::overwrite_existing);
    joint_sensor_player::AsyncReport report(run_dir);
    for (const auto& annotation : catalog.annotations) {
      report.add_anomaly(annotation.kind,
                         annotation.topic + ": " + annotation.detail);
    }
    report.log("mcap_files=" + std::to_string(catalog.files.size()));
    report.log("playback_duration_ns=" +
               std::to_string(window.source_end_ns - window.source_start_ns));
    std::cout << "网口: " << config.interface
              << " 1000Mbps Full Duplex，八个源IP已就绪\n";

    joint_sensor_player::ReportSummary summary;
    summary.files_scanned = catalog.files.size();
    summary.duplicate_messages = catalog.duplicate_messages;
    if (arguments.dry_run) {
      summary.ok = true;
      summary.stop_reason = "dry_run";
      report.log("dry_run=ok");
      report.finish(summary);
      std::cout << "DRY_RUN_OK，报告: " << run_dir << '\n';
      return 0;
    }

    joint_sensor_player::UdpDatagramSink sink(config);
    joint_sensor_player::MonotonicClock clock;
    joint_sensor_player::CancellationToken cancellation(&g_signal_stop);
    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);
    std::cout << "将在" << config.start_delay_ms
              << " ms后按严格1倍速开始发送\n";
    const auto result =
        joint_sensor_player::play_catalog(config, catalog, clock, sink, &report,
                                          &cancellation);
    report.log("packets_sent=" + std::to_string(result.packets_sent));
    report.log("frames_sent=" + std::to_string(result.frames_sent));
    summary.ok = result.ok;
    summary.frames_sent = result.frames_sent;
    summary.packets_sent = result.packets_sent;
    summary.max_lateness_ns = result.max_lateness_ns;
    summary.loops_completed = result.loops_completed;
    summary.stop_reason = result.stop_reason;
    summary.error = result.error;
    report.finish(summary);
    if (!result.ok) {
      std::cerr << "播放失败: " << result.error << '\n';
      return 3;
    }
    std::cout << "播放结束: loops=" << result.loops_completed
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
