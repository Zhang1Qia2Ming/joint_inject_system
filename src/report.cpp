#include "joint_sensor_player/report.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
#include <stdexcept>
#include <utility>

namespace joint_sensor_player {

AsyncReport::AsyncReport(std::filesystem::path output_dir,
                         std::size_t queue_capacity)
    : output_dir_(std::move(output_dir)), capacity_(queue_capacity) {
  std::filesystem::create_directories(output_dir_);
  frame_file_.open(output_dir_ / "frame_stats.csv");
  anomaly_file_.open(output_dir_ / "anomalies.jsonl");
  log_file_.open(output_dir_ / "run.log");
  if (!frame_file_ || !anomaly_file_ || !log_file_) {
    throw std::runtime_error("无法创建报告文件");
  }
  frame_file_ << "slot,topic,source_timestamp_ns,planned_last_packet_ns,"
                 "actual_last_packet_ns,packet_count,lateness_ns\n";
  worker_ = std::thread(&AsyncReport::worker_loop, this);
}

AsyncReport::~AsyncReport() {
  if (!finished_) {
    ReportSummary summary;
    summary.error = "report closed without explicit finish";
    finish(summary);
  }
}

bool AsyncReport::try_submit(FrameStat stat) {
  if (capacity_ == 0 || !mutex_.try_lock()) {
    ++dropped_;
    return false;
  }
  std::unique_lock<std::mutex> lock(mutex_, std::adopt_lock);
  if (stopping_ || queue_.size() >= capacity_) {
    ++dropped_;
    return false;
  }
  queue_.push(std::move(stat));
  lock.unlock();
  condition_.notify_one();
  return true;
}

void AsyncReport::add_anomaly(const std::string& type,
                              const std::string& detail) {
  nlohmann::json value{{"type", type}, {"detail", detail}};
  std::lock_guard<std::mutex> lock(mutex_);
  anomaly_file_ << value.dump() << '\n';
}

void AsyncReport::log(const std::string& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  log_file_ << message << '\n';
}

void AsyncReport::worker_loop() {
  while (true) {
    FrameStat stat;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
      if (queue_.empty() && stopping_) break;
      stat = std::move(queue_.front());
      queue_.pop();
    }
    frame_file_ << stat.slot << ',' << stat.topic << ','
                << stat.source_timestamp_ns << ','
                << stat.planned_last_packet_ns << ','
                << stat.actual_last_packet_ns << ',' << stat.packet_count << ','
                << stat.lateness_ns << '\n';
  }
}

void AsyncReport::finish(const ReportSummary& summary) {
  if (finished_) return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  condition_.notify_all();
  if (worker_.joinable()) worker_.join();
  frame_file_.flush();
  anomaly_file_.flush();
  log_file_.flush();

  nlohmann::json value{
      {"schema_version", 1},
      {"ok", summary.ok},
      {"files_scanned", summary.files_scanned},
      {"frames_sent", summary.frames_sent},
      {"packets_sent", summary.packets_sent},
      {"max_lateness_ns", summary.max_lateness_ns},
      {"duplicate_messages", summary.duplicate_messages},
      {"loops_completed", summary.loops_completed},
      {"stop_reason", summary.stop_reason},
      {"report_records_dropped", dropped_.load()},
      {"error", summary.error},
  };
  const auto temporary = output_dir_ / "summary.json.tmp";
  {
    std::ofstream output(temporary);
    if (!output) throw std::runtime_error("无法创建summary.json");
    output << value.dump(2) << '\n';
  }
  std::filesystem::rename(temporary, output_dir_ / "summary.json");
  finished_ = true;
}

}  // namespace joint_sensor_player
