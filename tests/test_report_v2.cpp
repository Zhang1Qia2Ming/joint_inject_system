#include "joint_sensor_orchestrator/report_v2.hpp"
#include "joint_sensor_orchestrator/report_failure_guard.hpp"
#include "test_framework.hpp"

#include <nlohmann/json.hpp>
#include <unistd.h>

#include <filesystem>
#include <fstream>

namespace {

std::filesystem::path report_test_dir(const char* name) {
  return std::filesystem::temp_directory_path() /
         (std::string(name) + "-" + std::to_string(::getpid()));
}

nlohmann::json read_json(const std::filesystem::path& path) {
  std::ifstream input(path);
  return nlohmann::json::parse(input);
}

}  // namespace

TEST_CASE(report_v2_is_independent_and_marks_only_mock_software_pass) {
  const auto root = report_test_dir("report-v2-layers");
  std::filesystem::remove_all(root);
  joint_sensor_orchestrator::ReportV2 report(root);
  joint_sensor_orchestrator::ReportV2Summary summary;
  summary.ok = true;
  summary.validation.software_logic =
      joint_sensor_orchestrator::ValidationStatus::Pass;
  summary.backend = "mock";
  summary.payload_mode = "synthetic";
  summary.final_state = joint_sensor_orchestrator::LifecycleState::Stopped;
  summary.common_window = {100, 200};
  summary.play_start_ns = 1'000;
  summary.utc_anchor_us = 2'000;
  summary.source_metadata_changed = true;
  summary.source_manifest_mtime_ns = 10;
  summary.source_current_mtime_ns = 20;
  summary.radar_streaming_bounds = {81, 128, 3528, 8192};
  report.finish(summary);

  const auto json = read_json(root / "summary.json");
  CHECK(json["schema_version"] == 2);
  CHECK(json["validation"]["software_logic"] == "PASS");
  CHECK(json["validation"]["video_payload_io"] == "NOT_RUN");
  CHECK(json["validation"]["xdma_device"] == "NOT_RUN");
  CHECK(json["validation"]["thor_nvsipl"] == "NOT_RUN");
  CHECK(json["validation"]["physical_sync"] == "NOT_RUN");
  CHECK(json["validation"]["hardware_fsync"] == "NOT_RUN");
  CHECK(json["backend"] == "mock");
  CHECK(json["payload_mode"] == "synthetic");
  CHECK(json["common_window"]["source_start_ns"] == 100);
  CHECK(json["anchors"]["play_start_ns"] == 1'000);
  CHECK(json["source_metadata"]["changed"] == true);
  CHECK(json["source_metadata"]["manifest_mtime_ns"] == 10);
  CHECK(json["source_metadata"]["current_mtime_ns"] == 20);
  CHECK(json["radar_streaming_bounds"]["required_reorder_frames"] == 81);
  CHECK(json["radar_streaming_bounds"]["limit_reorder_frames"] == 128);
  CHECK(json["radar_streaming_bounds"]["required_packet_heap"] == 3528);
  CHECK(json["radar_streaming_bounds"]["limit_packet_heap"] == 8192);
  std::filesystem::remove_all(root);
}

TEST_CASE(report_v2_uses_explicit_not_run_and_fail_validation_states) {
  const auto root = report_test_dir("report-v2-explicit-validation");
  std::filesystem::remove_all(root);
  joint_sensor_orchestrator::ReportV2 report(root);
  joint_sensor_orchestrator::ReportV2Summary summary;
  summary.ok = true;
  summary.final_state = joint_sensor_orchestrator::LifecycleState::Stopped;
  summary.validation.software_logic =
      joint_sensor_orchestrator::ValidationStatus::NotRun;
  report.finish(summary);
  CHECK(read_json(root / "summary.json")["validation"]["software_logic"] ==
        "NOT_RUN");
  std::filesystem::remove_all(root);

  const auto failed_root = report_test_dir("report-v2-explicit-fail");
  std::filesystem::remove_all(failed_root);
  joint_sensor_orchestrator::ReportV2 failed_report(failed_root);
  summary.ok = false;
  summary.validation.software_logic =
      joint_sensor_orchestrator::ValidationStatus::Fail;
  failed_report.finish(summary);
  CHECK(read_json(failed_root / "summary.json")["validation"]["software_logic"] ==
        "FAIL");
  std::filesystem::remove_all(failed_root);
}

TEST_CASE(report_v2_cancelled_real_run_is_incomplete_not_pass) {
  const auto root = report_test_dir("report-v2-cancelled");
  std::filesystem::remove_all(root);
  joint_sensor_orchestrator::ReportV2 report(root);
  joint_sensor_orchestrator::ReportV2Summary summary;
  summary.ok = true;
  summary.cancelled = true;
  summary.final_state = joint_sensor_orchestrator::LifecycleState::Stopped;
  summary.validation.software_logic =
      joint_sensor_orchestrator::ValidationStatus::Incomplete;
  report.finish(summary);
  const auto json = read_json(root / "summary.json");
  CHECK(json["validation"]["software_logic"] == "INCOMPLETE");
  CHECK(json["validation"]["xdma_device"] == "NOT_RUN");
  std::filesystem::remove_all(root);
}

TEST_CASE(report_v2_failed_atomic_publish_can_be_retried) {
  const auto root = report_test_dir("report-v2-retry");
  std::filesystem::remove_all(root);
  unsigned attempts = 0;
  joint_sensor_orchestrator::ReportV2Options options;
  options.before_summary_publish =
      [&](const std::filesystem::path&) {
        if (++attempts == 1) throw std::runtime_error("injected publish failure");
      };
  joint_sensor_orchestrator::ReportV2 report(root, std::move(options));
  joint_sensor_orchestrator::ReportV2Summary summary;
  summary.validation.software_logic =
      joint_sensor_orchestrator::ValidationStatus::Fail;
  CHECK_THROWS(report.finish(summary));
  CHECK(report.finish_count() == 0);
  report.finish(summary);
  CHECK(report.finish_count() == 1);
  CHECK(attempts == 2);
  CHECK(std::filesystem::is_regular_file(root / "summary.json"));
  std::filesystem::remove_all(root);
}

TEST_CASE(report_v2_records_lane_mapping_lateness_waterline_and_first_failure) {
  const auto root = report_test_dir("report-v2-evidence");
  std::filesystem::remove_all(root);
  joint_sensor_orchestrator::ReportV2 report(root);
  report.record_video_frame({
      "tx1", 7, 21, 300, 333, 1'333, 1'336, 3});
  joint_sensor_orchestrator::ReportV2Summary summary;
  summary.backend = "mock";
  summary.final_state = joint_sensor_orchestrator::LifecycleState::Stopped;
  summary.first_failure = {"video", "late"};
  summary.additional_failures.push_back({"radar", "cancelled"});
  summary.lanes[0] = {"tx1", 8, 24, 5, 3, 9, 1, 2};
  report.finish(summary);
  report.finish(summary);

  const auto json = read_json(root / "summary.json");
  CHECK(json["lanes"][0]["source_frames"] == 8);
  CHECK(json["lanes"][0]["output_frames"] == 24);
  CHECK(json["lanes"][0]["queue_max_waterline"] == 5);
  CHECK(json["lanes"][0]["lateness_ns"]["max"] == 9);
  CHECK(json["first_failure"]["component"] == "video");
  CHECK(json["additional_failures"][0]["component"] == "radar");
  CHECK(report.finish_count() == 1);

  std::ifstream frames(root / "video_frames.csv");
  std::string header;
  std::string row;
  std::getline(frames, header);
  std::getline(frames, row);
  CHECK(header.find("source_index") != std::string::npos);
  CHECK(row.find("tx1,7,21,300,333,1333,1336,3") != std::string::npos);
  CHECK(std::filesystem::is_regular_file(root / "frame_stats.csv"));
  CHECK(std::filesystem::is_regular_file(root / "anomalies.jsonl"));
  CHECK(std::filesystem::is_regular_file(root / "run.log"));
  std::filesystem::remove_all(root);
}

TEST_CASE(report_guard_finishes_once_for_udp_sink_and_engine_construction_failures) {
  for (const std::string component : {"udp", "sink", "engine"}) {
    const auto root = report_test_dir(("report-guard-" + component).c_str());
    std::filesystem::remove_all(root);
    joint_sensor_orchestrator::ReportV2 report(root);
    {
      joint_sensor_orchestrator::ReportV2Summary base;
      joint_sensor_orchestrator::ReportFailureGuard guard(report, base);
      guard.mark_execution_attempted();
      try {
        throw std::runtime_error(component + " construction failed");
      } catch (const std::exception& error) {
        guard.set_failure(component, error.what());
      }
    }
    const auto json = read_json(root / "summary.json");
    CHECK(json["validation"]["software_logic"] == "FAIL");
    CHECK(json["first_failure"]["component"] == component);
    CHECK(report.finish_count() == 1);
    std::filesystem::remove_all(root);
  }
}
