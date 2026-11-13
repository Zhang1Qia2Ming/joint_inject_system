#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace joint_sensor_player {

struct FrameStat {
  int slot{};
  std::string topic;
  std::uint64_t source_timestamp_ns{};
  std::uint64_t planned_last_packet_ns{};
  std::uint64_t actual_last_packet_ns{};
  std::uint32_t packet_count{};
  std::uint64_t lateness_ns{};
};

struct ReportSummary {
  bool ok{};
  std::size_t files_scanned{};
  std::uint64_t frames_sent{};
  std::uint64_t packets_sent{};
  std::uint64_t max_lateness_ns{};
  std::size_t duplicate_messages{};
  std::uint64_t loops_completed{};
  std::string stop_reason;
  std::string error;
};

class AsyncReport {
 public:
  explicit AsyncReport(std::filesystem::path output_dir,
                       std::size_t queue_capacity = 4096);
  ~AsyncReport();
  AsyncReport(const AsyncReport&) = delete;
  AsyncReport& operator=(const AsyncReport&) = delete;

  bool try_submit(FrameStat stat);
  void add_anomaly(const std::string& type, const std::string& detail);
  void log(const std::string& message);
  void finish(const ReportSummary& summary);
  std::uint64_t dropped_records() const { return dropped_.load(); }
  const std::filesystem::path& output_dir() const { return output_dir_; }

 private:
  void worker_loop();

  std::filesystem::path output_dir_;
  std::size_t capacity_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::queue<FrameStat> queue_;
  bool stopping_{};
  bool finished_{};
  std::atomic<std::uint64_t> dropped_{};
  std::thread worker_;
  std::ofstream frame_file_;
  std::ofstream anomaly_file_;
  std::ofstream log_file_;
};

}  // namespace joint_sensor_player
