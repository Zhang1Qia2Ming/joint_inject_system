#include "joint_sensor_player/report.hpp"
#include "test_framework.hpp"

#include <filesystem>
#include <fstream>
#include <iterator>

TEST_CASE(report_writes_stable_summary_and_frame_csv) {
  const auto dir =
      std::filesystem::temp_directory_path() / "joint_player_report";
  std::filesystem::remove_all(dir);
  {
    joint_sensor_player::AsyncReport report(dir, 8);
    CHECK(report.try_submit(
        {31, "/sensor/lidar_front/orig/udp_packets", 1000, 2000, 2100, 626,
         10}));
    CHECK(report.try_submit(
        {35, "/sensor/short_lidar_front/orig/udp_packets", 1100, 2200, 2250,
         512, 5}));
    report.add_anomaly("segment_gap", "gap detected");
    report.log("dry-run ready");
    joint_sensor_player::ReportSummary summary;
    summary.ok = true;
    summary.files_scanned = 3;
    summary.packets_sent = 1138;
    summary.duplicate_messages = 1;
    summary.loops_completed = 2;
    summary.stop_reason = "completed";
    report.finish(summary);
  }
  CHECK(std::filesystem::exists(dir / "summary.json"));
  CHECK(std::filesystem::exists(dir / "frame_stats.csv"));
  CHECK(std::filesystem::exists(dir / "anomalies.jsonl"));
  CHECK(std::filesystem::exists(dir / "run.log"));
  std::ifstream summary_file(dir / "summary.json");
  const std::string summary((std::istreambuf_iterator<char>(summary_file)),
                            std::istreambuf_iterator<char>());
  CHECK(summary.find("\"packets_sent\": 1138") != std::string::npos);
  CHECK(summary.find("\"schema_version\": 1") != std::string::npos);
  CHECK(summary.find("\"loops_completed\": 2") != std::string::npos);
  CHECK(summary.find("\"stop_reason\": \"completed\"") !=
        std::string::npos);
  std::ifstream csv_file(dir / "frame_stats.csv");
  const std::string csv((std::istreambuf_iterator<char>(csv_file)),
                        std::istreambuf_iterator<char>());
  CHECK(csv.find("slot,topic,source_timestamp_ns") != std::string::npos);
  CHECK(csv.find("31,/sensor/lidar_front/orig/udp_packets") !=
        std::string::npos);
  std::filesystem::remove_all(dir);
}

TEST_CASE(report_queue_overflow_never_blocks_and_is_counted) {
  const auto dir =
      std::filesystem::temp_directory_path() / "joint_player_report_overflow";
  std::filesystem::remove_all(dir);
  joint_sensor_player::AsyncReport report(dir, 0);
  CHECK(!report.try_submit({31, "topic", 2, 3, 4, 5, 6}));
  joint_sensor_player::ReportSummary summary;
  report.finish(summary);
  CHECK(report.dropped_records() == 1);
  std::filesystem::remove_all(dir);
}
