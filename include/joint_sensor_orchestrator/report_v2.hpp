#pragma once

// 一次联合播放的报告。发送线程只把一行放进队列，写 CSV 和 summary.json
// 在单独的 writer 线程里做，不占写卡和雷达发送的时间。
// 抛出门限的那一帧来不及入队，所以 CSV 里的最大迟到会小于真正停掉的那一次。

#include "joint_sensor_orchestrator/lifecycle_manager.hpp"
#include "joint_sensor_orchestrator/unified_timeline.hpp"

#include <array>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace joint_sensor_orchestrator {

struct VideoFrameEvidence {
  std::string lane;
  std::uint64_t source_index{};
  std::uint64_t output_index{};
  std::uint64_t source_time_ns{};
  std::uint64_t slot_time_ns{};
  std::uint64_t planned_deadline_ns{};
  // 写卡前采样到的醒来时刻和迟到。不是 write 返回之后的时刻。
  std::uint64_t actual_write_ns{};
  std::uint64_t lateness_ns{};
};

struct LaneReportV2 {
  std::string name;
  std::uint64_t source_frames{};
  std::uint64_t output_frames{};
  std::size_t queue_max_waterline{};
  std::uint64_t lateness_min_ns{};
  std::uint64_t lateness_max_ns{};
  std::uint64_t lateness_sum_ns{};
  std::uint64_t late_frames{};
  std::uint64_t lateness_samples{};
};

struct LifecycleTimingV2 {
  LifecycleState state{LifecycleState::Created};
  std::uint64_t entered_monotonic_ns{};
  std::uint64_t duration_ns{};
};

enum class ValidationStatus {
  NotRun,
  Pass,
  Fail,
  Incomplete,
};

struct ValidationLayersV2 {
  ValidationStatus software_logic{ValidationStatus::NotRun};
  ValidationStatus video_payload_io{ValidationStatus::NotRun};
  ValidationStatus xdma_device{ValidationStatus::NotRun};
  ValidationStatus thor_nvsipl{ValidationStatus::NotRun};
  ValidationStatus physical_sync{ValidationStatus::NotRun};
  ValidationStatus hardware_fsync{ValidationStatus::NotRun};
};

struct RadarStreamingBoundsV2 {
  std::size_t required_reorder_frames{};
  std::size_t limit_reorder_frames{};
  std::size_t required_packet_heap{};
  std::size_t limit_packet_heap{};
};

struct ReportV2Summary {
  bool ok{};
  bool cancelled{};
  std::string backend{"mock"};
  std::string payload_mode{"unknown"};
  LifecycleState final_state{LifecycleState::Created};
  SourceWindow common_window;
  std::uint64_t play_start_ns{};
  std::uint64_t utc_anchor_us{};
  std::array<LaneReportV2, 3> lanes;
  std::vector<LifecycleTimingV2> lifecycle;
  LifecycleFailure first_failure;
  std::vector<LifecycleFailure> additional_failures;
  std::string stop_reason;
  ValidationLayersV2 validation;
  std::vector<std::string> radar_participants;
  std::vector<std::string> video_participants;
  bool source_metadata_changed{};
  std::uint64_t source_manifest_mtime_ns{};
  std::uint64_t source_current_mtime_ns{};
  RadarStreamingBoundsV2 radar_streaming_bounds;
};

struct ReportV2Options {
  std::function<void(const std::filesystem::path&)>
      before_summary_publish;
};

class ReportV2 {
 public:
  explicit ReportV2(std::filesystem::path output_dir,
                    ReportV2Options options = {});
  ~ReportV2() noexcept;
  ReportV2(const ReportV2&) = delete;
  ReportV2& operator=(const ReportV2&) = delete;

  // 发送线程调用。只入队，不在这里写盘。
  void record_video_frame(const VideoFrameEvidence& frame);
  void record_radar_frame(const std::string& csv_row);
  void add_anomaly(const std::string& type, const std::string& detail);
  void log(const std::string& message);
  // 把 summary.json 写完并停掉 writer。失败路径也会走到这里。
  void finish(const ReportV2Summary& summary);

  const std::filesystem::path& output_dir() const noexcept {
    return output_dir_;
  }
  unsigned finish_count() const noexcept;

 private:
  enum class RecordKind { Video, Radar, Anomaly, Log };
  struct Record {
    RecordKind kind{};
    std::string text;
  };

  void enqueue(Record record);
  void writer_main();
  void drain_writer();
  void stop_writer() noexcept;

  std::filesystem::path output_dir_;
  mutable std::mutex mutex_;
  std::condition_variable pending_cv_;
  std::deque<Record> pending_;
  std::thread writer_;
  std::ofstream video_frames_;
  std::ofstream radar_frames_;
  std::ofstream anomalies_;
  std::ofstream log_;
  bool finished_{};
  bool finishing_{};
  bool writer_busy_{};
  bool stop_writer_{};
  unsigned finish_count_{};
  ReportV2Options options_;
};

std::string lifecycle_state_name(LifecycleState state);
std::string validation_status_name(ValidationStatus status);

}  // namespace joint_sensor_orchestrator
