#include "joint_sensor_orchestrator/report_v2.hpp"

#include <nlohmann/json.hpp>

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace joint_sensor_orchestrator {
namespace {

using Json = nlohmann::json;

Json failure_json(const LifecycleFailure& failure) {
  if (failure.component.empty()) return nullptr;
  return {{"component", failure.component}, {"reason", failure.reason}};
}

void atomic_write_summary(const std::filesystem::path& output_dir,
                          const std::string& contents,
                          const ReportV2Options& options) {
  const auto temporary =
      output_dir / (".summary.json.tmp." + std::to_string(::getpid()));
  const int descriptor = ::open(
      temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (descriptor < 0) {
    throw std::runtime_error("无法创建schema-v2 summary临时文件: " +
                             std::string(std::strerror(errno)));
  }
  bool closed = false;
  try {
    std::size_t offset = 0;
    while (offset < contents.size()) {
      const auto written =
          ::write(descriptor, contents.data() + offset, contents.size() - offset);
      if (written < 0 && errno == EINTR) continue;
      if (written <= 0) {
        throw std::runtime_error("写入schema-v2 summary临时文件失败");
      }
      offset += static_cast<std::size_t>(written);
    }
    if (::fsync(descriptor) != 0) {
      throw std::runtime_error("fsync schema-v2 summary临时文件失败");
    }
    if (::close(descriptor) != 0) {
      throw std::runtime_error("关闭schema-v2 summary临时文件失败");
    }
    closed = true;
    if (options.before_summary_publish) {
      options.before_summary_publish(temporary);
    }
    if (::rename(temporary.c_str(), (output_dir / "summary.json").c_str()) !=
        0) {
      throw std::runtime_error("原子发布schema-v2 summary失败: " +
                               std::string(std::strerror(errno)));
    }
    const int directory =
        ::open(output_dir.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (directory >= 0) {
      (void)::fsync(directory);
      (void)::close(directory);
    }
  } catch (...) {
    if (!closed) (void)::close(descriptor);
    (void)::unlink(temporary.c_str());
    throw;
  }
}

}  // namespace

std::string lifecycle_state_name(LifecycleState state) {
  switch (state) {
    case LifecycleState::Created:
      return "CREATED";
    case LifecycleState::Preparing:
      return "PREPARING";
    case LifecycleState::Ready:
      return "READY";
    case LifecycleState::Armed:
      return "ARMED";
    case LifecycleState::Running:
      return "RUNNING";
    case LifecycleState::Failed:
      return "FAILED";
    case LifecycleState::Stopping:
      return "STOPPING";
    case LifecycleState::Stopped:
      return "STOPPED";
  }
  throw std::logic_error("未知生命周期状态");
}

std::string validation_status_name(ValidationStatus status) {
  switch (status) {
    case ValidationStatus::NotRun:
      return "NOT_RUN";
    case ValidationStatus::Pass:
      return "PASS";
    case ValidationStatus::Fail:
      return "FAIL";
    case ValidationStatus::Incomplete:
      return "INCOMPLETE";
  }
  throw std::logic_error("未知验证状态");
}

ReportV2::ReportV2(std::filesystem::path output_dir, ReportV2Options options)
    : output_dir_(std::move(output_dir)), options_(std::move(options)) {
  std::filesystem::create_directories(output_dir_);
  video_frames_.open(output_dir_ / "video_frames.csv");
  radar_frames_.open(output_dir_ / "frame_stats.csv");
  anomalies_.open(output_dir_ / "anomalies.jsonl");
  log_.open(output_dir_ / "run.log");
  if (!video_frames_ || !radar_frames_ || !anomalies_ || !log_) {
    throw std::runtime_error("无法创建schema-v2报告文件");
  }
  video_frames_
      << "lane,source_index,output_index,source_time_ns,slot_time_ns,"
         "planned_deadline_ns,actual_write_ns,lateness_ns\n";
  radar_frames_
      << "slot,topic,source_timestamp_ns,planned_last_packet_ns,"
         "actual_last_packet_ns,packet_count,lateness_ns\n";
  // Stay busy until the writer has observed the queue, so finish() cannot
  // flush the files before that thread has started.
  writer_busy_ = true;
  writer_ = std::thread([this] { writer_main(); });
}

ReportV2::~ReportV2() noexcept {
  try {
    if (!finished_) {
      ReportV2Summary fallback;
      fallback.ok = false;
      fallback.final_state = LifecycleState::Failed;
      fallback.stop_reason = "report_scope_abandoned";
      fallback.first_failure = {"report", "报告作用域异常退出"};
      fallback.validation.software_logic = ValidationStatus::Fail;
      finish(fallback);
    }
  } catch (...) {
  }
  stop_writer();
}

void ReportV2::enqueue(Record record) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (finished_ || finishing_) return;
  pending_.push_back(std::move(record));
  pending_cv_.notify_one();
}

void ReportV2::writer_main() {
  while (true) {
    std::deque<Record> batch;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (pending_.empty()) {
        writer_busy_ = false;
        pending_cv_.notify_all();
        pending_cv_.wait(lock, [&] {
          return stop_writer_ || !pending_.empty();
        });
        if (stop_writer_ && pending_.empty()) return;
        if (pending_.empty()) continue;
      }
      batch.swap(pending_);
      writer_busy_ = true;
    }
    for (const auto& record : batch) {
      switch (record.kind) {
        case RecordKind::Video:
          video_frames_ << record.text;
          break;
        case RecordKind::Radar:
          radar_frames_ << record.text;
          break;
        case RecordKind::Anomaly:
          anomalies_ << record.text;
          break;
        case RecordKind::Log:
          log_ << record.text;
          break;
      }
    }
  }
}

void ReportV2::drain_writer() {
  std::unique_lock<std::mutex> lock(mutex_);
  pending_cv_.wait(lock, [&] {
    return pending_.empty() && !writer_busy_;
  });
}

void ReportV2::stop_writer() noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_writer_ = true;
  }
  pending_cv_.notify_all();
  if (writer_.joinable()) writer_.join();
}

void ReportV2::record_video_frame(const VideoFrameEvidence& frame) {
  std::ostringstream row;
  row << frame.lane << ',' << frame.source_index << ','
      << frame.output_index << ',' << frame.source_time_ns << ','
      << frame.slot_time_ns << ',' << frame.planned_deadline_ns << ','
      << frame.actual_write_ns << ',' << frame.lateness_ns << '\n';
  enqueue({RecordKind::Video, row.str()});
}

void ReportV2::record_radar_frame(const std::string& csv_row) {
  enqueue({RecordKind::Radar, csv_row + '\n'});
}

void ReportV2::add_anomaly(const std::string& type,
                           const std::string& detail) {
  enqueue({RecordKind::Anomaly,
           Json{{"type", type}, {"detail", detail}}.dump() + '\n'});
}

void ReportV2::log(const std::string& message) {
  enqueue({RecordKind::Log, message + '\n'});
}

void ReportV2::finish(const ReportV2Summary& summary) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (finished_) return;
    finishing_ = true;
  }
  pending_cv_.notify_all();
  drain_writer();

  Json lanes = Json::array();
  for (const auto& lane : summary.lanes) {
    if (lane.name.empty()) continue;
    lanes.push_back({
        {"name", lane.name},
        {"source_frames", lane.source_frames},
        {"output_frames", lane.output_frames},
        {"queue_max_waterline", lane.queue_max_waterline},
        {"lateness_ns",
         {{"min", lane.lateness_min_ns},
          {"max", lane.lateness_max_ns},
          {"sum", lane.lateness_sum_ns},
          {"late_frames", lane.late_frames},
          {"samples", lane.lateness_samples}}},
    });
  }
  Json lifecycle = Json::array();
  for (const auto& entry : summary.lifecycle) {
    lifecycle.push_back({
        {"state", lifecycle_state_name(entry.state)},
        {"entered_monotonic_ns", entry.entered_monotonic_ns},
        {"duration_ns", entry.duration_ns},
    });
  }
  Json additional = Json::array();
  for (const auto& failure : summary.additional_failures) {
    additional.push_back(failure_json(failure));
  }
  const Json document = {
      {"schema_version", 2},
      {"ok", summary.ok},
      {"cancelled", summary.cancelled},
      {"stop_reason", summary.stop_reason},
      {"backend", summary.backend},
      {"payload_mode", summary.payload_mode},
      {"lifecycle",
       {{"final_state", lifecycle_state_name(summary.final_state)},
        {"transitions", std::move(lifecycle)}}},
      {"common_window",
       {{"source_start_ns", summary.common_window.start_ns},
        {"source_end_ns", summary.common_window.end_ns}}},
      {"anchors",
       {{"play_start_ns", summary.play_start_ns},
        {"utc_anchor_us", summary.utc_anchor_us}}},
      {"lanes", std::move(lanes)},
      {"first_failure", failure_json(summary.first_failure)},
      {"additional_failures", std::move(additional)},
      {"participants",
       {{"radar_topics", summary.radar_participants},
        {"video_topics", summary.video_participants}}},
      {"source_metadata",
       {{"changed", summary.source_metadata_changed},
        {"manifest_mtime_ns", summary.source_manifest_mtime_ns},
        {"current_mtime_ns", summary.source_current_mtime_ns}}},
      {"radar_streaming_bounds",
       {{"required_reorder_frames",
         summary.radar_streaming_bounds.required_reorder_frames},
        {"limit_reorder_frames",
         summary.radar_streaming_bounds.limit_reorder_frames},
        {"required_packet_heap",
         summary.radar_streaming_bounds.required_packet_heap},
        {"limit_packet_heap",
         summary.radar_streaming_bounds.limit_packet_heap}}},
      {"validation",
       {{"software_logic",
         validation_status_name(summary.validation.software_logic)},
        {"video_payload_io",
         validation_status_name(summary.validation.video_payload_io)},
        {"xdma_device",
         validation_status_name(summary.validation.xdma_device)},
        {"thor_nvsipl",
         validation_status_name(summary.validation.thor_nvsipl)},
        {"physical_sync",
         validation_status_name(summary.validation.physical_sync)},
        {"hardware_fsync",
         validation_status_name(summary.validation.hardware_fsync)}}},
  };
  try {
    video_frames_.flush();
    radar_frames_.flush();
    anomalies_.flush();
    log_.flush();
    if (!video_frames_ || !radar_frames_ || !anomalies_ || !log_) {
      throw std::runtime_error("刷新schema-v2辅助报告文件失败");
    }
    atomic_write_summary(output_dir_, document.dump(2) + '\n', options_);
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    finishing_ = false;
    throw;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  finished_ = true;
  ++finish_count_;
}

unsigned ReportV2::finish_count() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return finish_count_;
}

}  // namespace joint_sensor_orchestrator
