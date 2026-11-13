#include "joint_sensor_orchestrator/process.hpp"
#include "joint_sensor_orchestrator/video_cache.hpp"
#include "joint_sensor_player/config.hpp"
#include "joint_sensor_player/packet_time_rewriter.hpp"
#include "test_data_builder.hpp"
#include "test_framework.hpp"

#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

namespace {

void write_le32(std::vector<std::uint8_t>& packet, std::size_t offset,
                std::uint32_t value) {
  std::memcpy(packet.data() + offset, &value, sizeof(value));
}

void write_seconds(std::vector<std::uint8_t>& packet, std::size_t offset,
                   std::uint32_t seconds) {
  packet[offset] = 0;
  packet[offset + 1] = 0;
  packet[offset + 2] = static_cast<std::uint8_t>(seconds >> 24U);
  packet[offset + 3] = static_cast<std::uint8_t>(seconds >> 16U);
  packet[offset + 4] = static_cast<std::uint8_t>(seconds >> 8U);
  packet[offset + 5] = static_cast<std::uint8_t>(seconds);
}

std::vector<std::uint8_t> valid_lidar_packet(
    bool main, std::uint64_t time_us, std::uint32_t sequence) {
  if (main) {
    std::vector<std::uint8_t> packet(1118);
    packet[0] = 0xEE;
    packet[1] = 0xFF;
    packet[2] = 0x04;
    packet[3] = 0x03;
    packet[6] = 128;
    packet[7] = 2;
    packet[11] = 1;
    write_le32(packet, 1066,
               static_cast<std::uint32_t>(time_us % 1'000'000));
    write_seconds(packet, 1072,
                  static_cast<std::uint32_t>(time_us / 1'000'000));
    write_le32(packet, 1078, sequence);
    write_le32(
        packet, 1082,
        joint_sensor_player::lidar_crc32(packet.data() + 1046, 36));
    return packet;
  }
  std::vector<std::uint8_t> packet(554);
  packet[0] = 0xEE;
  packet[1] = 0xFF;
  packet[2] = 0x07;
  packet[3] = 0x03;
  write_seconds(packet, 519,
                static_cast<std::uint32_t>(time_us / 1'000'000));
  write_le32(packet, 525,
             static_cast<std::uint32_t>(time_us % 1'000'000));
  packet[529] = 0x42;
  write_le32(packet, 530, sequence);
  return packet;
}

std::string sha256(const std::filesystem::path& path) {
  using Context =
      std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
  Context context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) !=
                      1) {
    throw std::runtime_error("sha init failed");
  }
  std::ifstream input(path, std::ios::binary);
  std::array<char, 64 * 1024> bytes{};
  while (input) {
    input.read(bytes.data(), bytes.size());
    if (input.gcount() > 0) {
      (void)EVP_DigestUpdate(
          context.get(), bytes.data(),
          static_cast<std::size_t>(input.gcount()));
    }
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned size{};
  (void)EVP_DigestFinal_ex(context.get(), digest.data(), &size);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned index = 0; index < size; ++index) {
    output << std::setw(2) << static_cast<unsigned>(digest[index]);
  }
  return output.str();
}

std::string sha256_text(const std::string& text) {
  const auto path = std::filesystem::temp_directory_path() /
                    ("sha-text-" + std::to_string(::getpid()));
  {
    std::ofstream output(path);
    output << text;
  }
  const auto result = sha256(path);
  std::filesystem::remove(path);
  return result;
}

std::uint64_t mtime_ns(const std::filesystem::path& path) {
  struct stat status {};
  if (::stat(path.c_str(), &status) != 0) throw std::runtime_error("stat");
  return static_cast<std::uint64_t>(status.st_mtim.tv_sec) *
             1'000'000'000ULL +
         static_cast<std::uint64_t>(status.st_mtim.tv_nsec);
}

class CliFixture {
 public:
  CliFixture()
      : root(std::filesystem::temp_directory_path() /
             ("orchestrator-cli-" + std::to_string(::getpid()))) {
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(data);
    std::filesystem::create_directories(cache_root);
    std::filesystem::create_directories(reports);
    write_radar();
    write_cache();
    write_config();
  }
  ~CliFixture() { std::filesystem::remove_all(root); }

  std::filesystem::path newest_report() const {
    std::filesystem::path result;
    for (const auto& entry : std::filesystem::directory_iterator(reports)) {
      if (entry.is_directory() &&
          (result.empty() || entry.path().filename() > result.filename())) {
        result = entry.path();
      }
    }
    if (result.empty()) throw std::runtime_error("missing report");
    return result;
  }

  nlohmann::json newest_summary() const {
    std::ifstream input(newest_report() / "summary.json");
    return nlohmann::json::parse(input);
  }

  void make_udp_construction_fail() {
    std::ifstream input(config);
    std::string text((std::istreambuf_iterator<char>(input)),
                     std::istreambuf_iterator<char>());
    const auto position = text.find("interface: lo");
    text.replace(position, std::string("interface: lo").size(),
                 "interface: definitely_missing_interface");
    std::ofstream output(config);
    output << text;
  }

  void make_engine_prepare_fail() {
    const auto channels = joint_sensor_player::default_lidar_channels();
    for (std::size_t index = 0; index < channels.size(); ++index) {
      const auto& channel = channels[index];
      std::vector<std::vector<std::uint8_t>> messages;
      for (unsigned frame = 0; frame < 2; ++frame) {
        std::vector<std::vector<std::uint8_t>> packets;
        const bool main = index < 4;
        const std::uint64_t step_us = main ? 83 : 193;
        for (std::uint32_t packet = 0;
             packet < channel.packets_per_frame; ++packet) {
          packets.push_back(valid_lidar_packet(
              main, 1'000'000ULL + frame * 100'000ULL +
                        packet * step_us,
              frame * channel.packets_per_frame + packet + 1));
        }
        messages.push_back(testdata::message(
            1, frame + 1, 1'000'000'000ULL,
            testdata::udp_frame_packets(
                1, frame * 100'000'000U, packets)));
      }
      testdata::write_mcap(
          data / ("lane-" + std::to_string(index) + ".mcap"),
          channel.topic, messages, false);
    }
  }

  std::filesystem::path root;
  std::filesystem::path data{root / "radar"};
  std::filesystem::path cache_root{root / "cache"};
  std::filesystem::path reports{root / "reports"};
  std::filesystem::path video{root / "video.mcap"};
  std::filesystem::path config{root / "config.yaml"};
  std::filesystem::path cache_key_dir;

 private:
  void write_radar() {
    const auto channels = joint_sensor_player::default_lidar_channels();
    for (std::size_t index = 0; index < channels.size(); ++index) {
      const auto& channel = channels[index];
      std::vector<std::vector<std::uint8_t>> messages;
      for (unsigned frame = 0; frame < 2; ++frame) {
        std::vector<std::vector<std::uint8_t>> packets;
        packets.reserve(channel.packets_per_frame);
        const bool main = index < 4;
        const std::uint64_t packet_step_us = main ? 83 : 193;
        for (std::uint32_t packet = 0;
             packet < channel.packets_per_frame; ++packet) {
          packets.push_back(valid_lidar_packet(
              main, 1'000'000ULL + frame * 100'000ULL +
                        packet * packet_step_us,
              frame * channel.packets_per_frame + packet + 1));
        }
        const auto payload = testdata::udp_frame_packets(
            1, frame * 100'000'000U, packets);
        messages.push_back(testdata::message(
            1, frame + 1, 1'000'000'000ULL + frame * 100'000'000ULL,
            payload));
      }
      testdata::write_mcap(
          data / ("lane-" + std::to_string(index) + ".mcap"),
          channel.topic, messages, false);
    }
  }

  void write_cache() {
    {
      std::ofstream output(video, std::ios::binary);
      output << "fixture-video";
    }
    const std::string dataset = sha256(video);
    const std::array<std::string, 3> topics{
        "/camera/tx1/h265", "/camera/tx2/h265", "/camera/tx4/h265"};
    const nlohmann::json identity = {
        {"converter_version", "joint-sensor-video-cache-1"},
        {"dataset_id", dataset},
        {"height", 2160},
        {"pixel_format", "uyvy422"},
        {"topics", topics},
        {"width", 3840},
    };
    const std::string key = sha256_text(identity.dump());
    const auto key_dir = cache_root / key;
    cache_key_dir = key_dir;
    std::filesystem::create_directory(key_dir);
    const std::array<std::string, 3> names{"tx1", "tx2", "tx4"};
    const std::array<unsigned, 3> rates{10, 30, 10};
    nlohmann::json manifest = {
        {"version", 1},
        {"dataset_id", dataset},
        {"cache_key", key},
        {"converter_version", "joint-sensor-video-cache-1"},
        {"source_mcap",
         {{"path", std::filesystem::canonical(video).string()},
          {"size", std::filesystem::file_size(video)},
          {"mtime_ns", mtime_ns(video)},
          {"sha256", dataset}}},
        {"width", 3840},
        {"height", 2160},
        {"pixel_format", "uyvy422"},
        {"lanes", nlohmann::json::object()},
    };
    for (std::size_t index = 0; index < names.size(); ++index) {
      const auto payload = key_dir / (names[index] + ".uyvy");
      {
        std::ofstream output(payload, std::ios::binary);
        std::vector<char> zeros(
            static_cast<std::size_t>(
                joint_sensor_orchestrator::kVideoFrameBytes),
            0);
        output.write(zeros.data(), zeros.size());
        output.write(zeros.data(), zeros.size());
      }
      const auto timestamps = key_dir / (names[index] + ".timestamps_ns");
      {
        std::ofstream output(timestamps);
        output << "1000000000\n1100000000\n";
      }
      manifest["lanes"][names[index]] = {
          {"topic", topics[index]},
          {"source_fps", rates[index]},
          {"output_fps", 30},
          {"pixel_format", "uyvy422"},
          {"frame_count", 2},
          {"frame_bytes", joint_sensor_orchestrator::kVideoFrameBytes},
          {"payload_file", names[index] + ".uyvy"},
          {"timestamps",
           {{"file", timestamps.filename().string()},
            {"byte_size", std::filesystem::file_size(timestamps)},
            {"sha256", sha256(timestamps)}}},
          {"sha256", sha256(payload)},
      };
    }
    std::ofstream output(key_dir / "manifest.json");
    output << manifest.dump(2);
  }

  void write_config() {
    std::ofstream output(config);
    output
        << "data_dir: " << data.string() << "\n"
        << "network:\n  interface: lo\n  destination_ip: 127.0.0.1\n"
        << "mode:\n  video_backend: mock\n  strict: true\n"
        << "playback:\n  loop_count: 1\n  start_delay_ms: 1000\n"
        << "clock:\n  require_synchronized: false\n"
        << "ptc:\n  enabled: false\n"
        << "report:\n  output_dir: " << reports.string() << "\n"
        << "video:\n  mcap: " << video.string() << "\n  cache_dir: "
        << cache_root.string() << "\n  queue_frames: 3\n  lanes:\n"
        << "    - {name: tx1, topic: /camera/tx1/h265, h2c: 0, source_fps: 10, output_fps: 30}\n"
        << "    - {name: tx2, topic: /camera/tx2/h265, h2c: 1, source_fps: 30, output_fps: 30}\n"
        << "    - {name: tx4, topic: /camera/tx4/h265, h2c: 2, source_fps: 10, output_fps: 30}\n"
        << "lidars:\n";
    for (int slot = 31; slot <= 38; ++slot) {
      output << "  - {slot: " << slot << ", source_ip: 127.0.0."
             << slot << "}\n";
    }
  }
};

bool has_lifecycle_state(const nlohmann::json& summary,
                         const std::string& expected) {
  for (const auto& transition :
       summary["lifecycle"]["transitions"]) {
    if (transition["state"] == expected) return true;
  }
  return false;
}

int run_and_signal(const std::filesystem::path& config, int signal) {
  const pid_t child = ::fork();
  if (child == 0) {
    const std::string binary =
        "/home/ubuntu/zhang/joint_sensor_player/build/joint_sensor_orchestrator";
    const std::string config_text = config.string();
    ::execl(binary.c_str(), binary.c_str(), "--config", config_text.c_str(),
            static_cast<char*>(nullptr));
    _exit(127);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  (void)::kill(child, signal);
  int status{};
  (void)::waitpid(child, &status, 0);
  return WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
}

}  // namespace

TEST_CASE(orchestrator_cli_dry_run_reports_not_run_and_all_participants) {
  CliFixture fixture;
  const auto result = joint_sensor_orchestrator::run_process_argv_capture_stdout(
      {"/home/ubuntu/zhang/joint_sensor_player/build/joint_sensor_orchestrator",
       "--config", fixture.config.string(), "--dry-run"});
  CHECK(result.exit_code == 0);
  const auto summary = fixture.newest_summary();
  CHECK(summary["validation"]["software_logic"] == "NOT_RUN");
  CHECK(summary["validation"]["video_payload_io"] == "NOT_RUN");
  CHECK(summary["backend"] == "mock");
  CHECK(summary["payload_mode"] == "synthetic");
  CHECK(summary["participants"]["radar_topics"].size() == 8);
  CHECK(summary["participants"]["video_topics"].size() == 3);
  CHECK(summary["lanes"][0]["source_frames"] == 1);
  CHECK(summary["lanes"][1]["source_frames"] == 2);
  CHECK(summary["lanes"][2]["source_frames"] == 1);
  CHECK(summary["radar_streaming_bounds"]["required_reorder_frames"] > 0);
  CHECK(summary["radar_streaming_bounds"]["limit_reorder_frames"] == 128);
  CHECK(summary["radar_streaming_bounds"]["required_packet_heap"] > 0);
  CHECK(summary["radar_streaming_bounds"]["limit_packet_heap"] == 8192);
}

TEST_CASE(orchestrator_cli_mock_pass_does_not_claim_video_payload_io) {
  CliFixture fixture;
  const auto result =
      joint_sensor_orchestrator::run_process_argv_capture_stdout(
          {"/home/ubuntu/zhang/joint_sensor_player/build/"
           "joint_sensor_orchestrator",
           "--config", fixture.config.string()});
  CHECK(result.exit_code == 0);
  const auto summary = fixture.newest_summary();
  CHECK(summary["schema_version"] == 2);
  CHECK(summary["backend"] == "mock");
  CHECK(summary["payload_mode"] == "synthetic");
  CHECK(summary["validation"]["software_logic"] == "PASS");
  CHECK(summary["validation"]["video_payload_io"] == "NOT_RUN");
  CHECK(summary["validation"]["xdma_device"] == "NOT_RUN");
  CHECK(summary["validation"]["thor_nvsipl"] == "NOT_RUN");
  CHECK(summary["validation"]["physical_sync"] == "NOT_RUN");
  CHECK(summary["validation"]["hardware_fsync"] == "NOT_RUN");
}

TEST_CASE(orchestrator_cli_prepare_cache_executes_stub_and_revalidates_manifest) {
  CliFixture fixture;
  const auto stub = fixture.root / "cache-builder-stub.py";
  {
    std::ofstream output(stub);
    output << "import pathlib\n"
              "print(pathlib.Path(r'"
           << fixture.cache_key_dir.string() << "').resolve())\n";
  }
  const auto result = joint_sensor_orchestrator::run_process_argv_capture_stdout(
      {"/usr/bin/env",
       "JOINT_SENSOR_CACHE_BUILDER=" + stub.string(),
       "/home/ubuntu/zhang/joint_sensor_player/build/joint_sensor_orchestrator",
       "--config", fixture.config.string(), "--prepare-cache"});
  CHECK(result.exit_code == 0);
  CHECK(result.stdout_text.find("CACHE_READY") != std::string::npos);
  CHECK(std::filesystem::is_regular_file(
      fixture.cache_key_dir / "manifest.json"));
}

TEST_CASE(orchestrator_cli_sigint_and_sigterm_stop_normally_without_hardware) {
  CliFixture fixture;
  CHECK(run_and_signal(fixture.config, SIGINT) == 0);
  auto summary = fixture.newest_summary();
  CHECK(summary["cancelled"] == true);
  CHECK(summary["validation"]["software_logic"] == "INCOMPLETE");
  CHECK(summary["lifecycle"]["final_state"] == "STOPPED");
  CHECK(has_lifecycle_state(summary, "RUNNING"));
  CHECK(std::filesystem::is_regular_file(
      fixture.newest_report() / "frame_stats.csv"));
  CHECK(std::filesystem::is_regular_file(
      fixture.newest_report() / "video_frames.csv"));
  CHECK(run_and_signal(fixture.config, SIGTERM) == 0);
  summary = fixture.newest_summary();
  CHECK(summary["cancelled"] == true);
  CHECK(summary["validation"]["software_logic"] == "INCOMPLETE");
  CHECK(summary["lifecycle"]["final_state"] == "STOPPED");
  CHECK(has_lifecycle_state(summary, "RUNNING"));
}

TEST_CASE(orchestrator_cli_udp_construction_failure_writes_one_failed_summary) {
  CliFixture fixture;
  fixture.make_udp_construction_fail();
  const auto result = joint_sensor_orchestrator::run_process_argv_capture_stdout(
      {"/home/ubuntu/zhang/joint_sensor_player/build/joint_sensor_orchestrator",
       "--config", fixture.config.string()});
  CHECK(result.exit_code == 2);
  const auto summary = fixture.newest_summary();
  CHECK(summary["validation"]["software_logic"] == "FAIL");
  CHECK(summary["lifecycle"]["final_state"] == "FAILED");
  CHECK(summary["first_failure"]["component"] == "main");
  CHECK(std::filesystem::is_regular_file(
      fixture.newest_report() / "summary.json"));
}

TEST_CASE(orchestrator_cli_engine_prepare_failure_writes_failed_summary) {
  CliFixture fixture;
  fixture.make_engine_prepare_fail();
  const auto result = joint_sensor_orchestrator::run_process_argv_capture_stdout(
      {"/home/ubuntu/zhang/joint_sensor_player/build/joint_sensor_orchestrator",
       "--config", fixture.config.string()});
  CHECK(result.exit_code == 3);
  const auto summary = fixture.newest_summary();
  CHECK(summary["validation"]["software_logic"] == "FAIL");
  CHECK(summary["first_failure"]["component"] == "radar");
}
