#include "joint_sensor_player/mcap.hpp"
#include "test_data_builder.hpp"
#include "test_framework.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using joint_sensor_player::default_lidar_channels;
using joint_sensor_player::scan_mcap_directory;
using joint_sensor_player::scan_mcap_directory_lightweight;
using joint_sensor_player::stream_mcap_frames;

namespace {

std::filesystem::path fresh_dir(const char* name) {
  const auto path = std::filesystem::temp_directory_path() / name;
  std::filesystem::remove_all(path);
  std::filesystem::create_directories(path);
  return path;
}

std::string thrown_message(const std::function<void()>& function) {
  try {
    function();
  } catch (const std::exception& error) {
    return error.what();
  }
  return {};
}

void write_oversized_chunk_header(const std::filesystem::path& path,
                                  std::uint64_t uncompressed_size) {
  std::vector<std::uint8_t> chunk;
  testdata::append_u64(chunk, 0);
  testdata::append_u64(chunk, 0);
  testdata::append_u64(chunk, uncompressed_size);
  testdata::append_u32(chunk, 0);
  testdata::append_string(chunk, "zstd");
  testdata::append_u64(chunk, 0);
  std::vector<std::uint8_t> bytes{
      0x89, 'M', 'C', 'A', 'P', '0', '\r', '\n'};
  testdata::append_record(bytes, 0x06, chunk);
  bytes.insert(bytes.end(),
               {0x89, 'M', 'C', 'A', 'P', '0', '\r', '\n'});
  std::ofstream output(path, std::ios::binary);
  output.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
}

void write_sparse_top_level_record(const std::filesystem::path& path,
                                   std::uint8_t opcode,
                                   std::uint64_t record_size) {
  constexpr std::array<std::uint8_t, 8> magic{
      0x89, 'M', 'C', 'A', 'P', '0', '\r', '\n'};
  std::ofstream output(path, std::ios::binary);
  output.write(reinterpret_cast<const char*>(magic.data()),
               static_cast<std::streamsize>(magic.size()));
  output.put(static_cast<char>(opcode));
  output.write(reinterpret_cast<const char*>(&record_size),
               sizeof(record_size));
  output.seekp(static_cast<std::streamoff>(record_size - 1),
               std::ios::cur);
  output.put('\0');
  output.write(reinterpret_cast<const char*>(magic.data()),
               static_cast<std::streamsize>(magic.size()));
}

}  // namespace

TEST_CASE(scan_merges_segments_by_source_time_and_deduplicates) {
  const auto dir = fresh_dir("joint_player_mcap_merge");
  const auto channel = default_lidar_channels()[0];
  const auto frame1 = testdata::udp_frame(1, 0, 1118, 2, 0x10);
  const auto frame2 = testdata::udp_frame(1, 100000000, 1118, 2, 0x20);
  const auto frame3 = testdata::udp_frame(1, 200000000, 1118, 2, 0x30);
  testdata::write_mcap(
      dir / "part1.mcap", channel.topic,
      {testdata::message(1, 1, 100, frame1),
       testdata::message(1, 2, 200, frame2)},
      false);
  testdata::write_mcap(
      dir / "part2.mcap", channel.topic,
      {testdata::message(1, 2, 200, frame2),
       testdata::message(1, 3, 300, frame3)},
      true);
  std::filesystem::create_directories(dir / "ignored");
  testdata::write_mcap(
      dir / "ignored" / "nested.mcap", channel.topic,
      {testdata::message(1, 4, 400,
                         testdata::udp_frame(1, 300000000, 1118, 1, 0x40))},
      false);

  const auto catalog = scan_mcap_directory(dir, {channel});
  std::filesystem::remove_all(dir);
  const auto& frames = catalog.frames.at(channel.topic);
  CHECK(catalog.files.size() == 2);
  CHECK(frames.size() == 3);
  CHECK(frames[0].source_timestamp_ns == 1000000000ULL);
  CHECK(frames[1].source_timestamp_ns == 1100000000ULL);
  CHECK(frames[2].source_timestamp_ns == 1200000000ULL);
  CHECK(frames[0].packets.size() == 2);
  CHECK(frames[0].packets[0].size() == 1118);
  CHECK(catalog.duplicate_messages == 1);
}

TEST_CASE(scan_decodes_fixed512_short_lidar) {
  const auto dir = fresh_dir("joint_player_mcap_short");
  const auto channel = default_lidar_channels()[4];
  testdata::write_mcap(
      dir / "short.mcap", channel.topic,
      {testdata::message(
          1, 1, 10, testdata::udp_frame(2, 50, 554, 3, 0x55))},
      true);
  const auto catalog = scan_mcap_directory(dir, {channel});
  std::filesystem::remove_all(dir);
  const auto& frame = catalog.frames.at(channel.topic).front();
  CHECK(frame.packets.size() == 3);
  CHECK(frame.packets[2].size() == 554);
  CHECK(frame.packets[2][0] == 0x57);
}

TEST_CASE(scan_rejects_conflicting_same_timestamp) {
  const auto dir = fresh_dir("joint_player_mcap_conflict");
  const auto channel = default_lidar_channels()[0];
  testdata::write_mcap(
      dir / "part1.mcap", channel.topic,
      {testdata::message(
          1, 1, 10, testdata::udp_frame(3, 0, 1118, 1, 0x10))},
      false);
  testdata::write_mcap(
      dir / "part2.mcap", channel.topic,
      {testdata::message(
          1, 2, 20, testdata::udp_frame(3, 0, 1118, 1, 0x20))},
      false);
  CHECK_THROWS(scan_mcap_directory(dir, {channel}));
  std::filesystem::remove_all(dir);
}

TEST_CASE(lightweight_scan_validates_packets_without_retaining_payload) {
  const auto dir = fresh_dir("joint_player_mcap_lightweight");
  const auto channel = default_lidar_channels()[0];
  const auto packet1 = testdata::main_lidar_packet(1'000'000, 10);
  const auto packet2 = testdata::main_lidar_packet(1'000'100, 11);
  testdata::write_mcap(
      dir / "radar.mcap", channel.topic,
      {testdata::message(
          1, 1, 100,
          testdata::udp_frame_packets(1, 0, {packet1, packet2}))},
      true);

  const auto catalog =
      scan_mcap_directory_lightweight(dir, {channel});
  CHECK(catalog.frames.at(channel.topic).size() == 1);
  CHECK(catalog.frames.at(channel.topic).front().packet_count == 2);
  CHECK(catalog.retained_payload_bytes == 0);
  CHECK(catalog.peak_chunk_bytes > 0);
  std::filesystem::remove_all(dir);
}

TEST_CASE(lightweight_scan_rejects_packet_time_or_sequence_rollback) {
  const auto dir = fresh_dir("joint_player_mcap_lightweight_rollback");
  const auto channel = default_lidar_channels()[0];
  testdata::write_mcap(
      dir / "radar.mcap", channel.topic,
      {testdata::message(
          1, 1, 100,
          testdata::udp_frame_packets(
              1, 0,
              {testdata::main_lidar_packet(1'000'100, 11),
               testdata::main_lidar_packet(1'000'000, 10)}))},
      false);
  CHECK_THROWS(scan_mcap_directory_lightweight(dir, {channel}));
  std::filesystem::remove_all(dir);
}

TEST_CASE(streaming_scan_orders_bounded_disorder_and_reports_hard_limit) {
  const auto dir = fresh_dir("joint_player_mcap_stream_order");
  const auto channel = default_lidar_channels()[0];
  const auto frame = [&](std::uint64_t time_us, std::uint32_t sequence) {
    return testdata::udp_frame_packets(
        1, static_cast<std::uint32_t>((time_us - 1'000'000) * 1000),
        {testdata::main_lidar_packet(time_us, sequence)});
  };
  testdata::write_mcap(
      dir / "radar.mcap", channel.topic,
      {testdata::message(1, 2, 200, frame(1'000'200, 12)),
       testdata::message(1, 1, 100, frame(1'000'100, 11)),
       testdata::message(1, 3, 300, frame(1'000'300, 13))},
      false);
  const auto catalog =
      scan_mcap_directory_lightweight(dir, {channel});
  CHECK(catalog.required_reorder_frames == 2);
  std::vector<std::uint64_t> observed;
  const auto stats = stream_mcap_frames(
      catalog, {channel}, {.max_reorder_frames = 3},
      [&](const std::string&, joint_sensor_player::LidarFrame frame) {
        observed.push_back(frame.bag_log_time_ns);
        return true;
      });
  CHECK(observed == std::vector<std::uint64_t>({100, 200, 300}));
  CHECK(stats.max_reorder_frames <= 3);
  CHECK_THROWS(stream_mcap_frames(
      catalog, {channel}, {.max_reorder_frames = 1},
      [](const std::string&, joint_sensor_player::LidarFrame) {
        return true;
      }));
  std::filesystem::remove_all(dir);
}

TEST_CASE(lightweight_scan_rejects_chunk_over_64_mib_before_allocation) {
  const auto dir = fresh_dir("joint_player_mcap_chunk_limit");
  write_oversized_chunk_header(
      dir / "oversized.mcap", 64ULL * 1024 * 1024 + 1);
  const auto error = thrown_message([&] {
    (void)scan_mcap_directory_lightweight(
        dir, {default_lidar_channels()[0]});
  });
  CHECK(error.find("64 MiB") != std::string::npos);
  std::filesystem::remove_all(dir);
}

TEST_CASE(lightweight_scan_rejects_top_level_message_over_64_mib_before_read) {
  const auto dir = fresh_dir("joint_player_mcap_message_limit");
  write_sparse_top_level_record(
      dir / "oversized.mcap", 0x05,
      64ULL * 1024 * 1024 + 1);
  const auto error = thrown_message([&] {
    (void)scan_mcap_directory_lightweight(
        dir, {default_lidar_channels()[0]});
  });
  CHECK(error.find("64 MiB") != std::string::npos);
  std::filesystem::remove_all(dir);
}

TEST_CASE(lightweight_scan_rejects_top_level_channel_over_64_mib_before_read) {
  const auto dir = fresh_dir("joint_player_mcap_channel_limit");
  write_sparse_top_level_record(
      dir / "oversized.mcap", 0x04,
      64ULL * 1024 * 1024 + 1);
  const auto error = thrown_message([&] {
    (void)scan_mcap_directory_lightweight(
        dir, {default_lidar_channels()[0]});
  });
  CHECK(error.find("64 MiB") != std::string::npos);
  std::filesystem::remove_all(dir);
}

TEST_CASE(streaming_scan_accepts_128_real_frames_at_reorder_limit) {
  const auto dir = fresh_dir("joint_player_mcap_reorder_128");
  const auto channel = default_lidar_channels()[0];
  std::vector<std::vector<std::uint8_t>> messages;
  for (std::uint32_t logical = 128; logical > 0; --logical) {
    messages.push_back(testdata::message(
        1, logical, 100 + logical,
        testdata::udp_frame_packets(
            1, logical * 1000,
            {testdata::main_lidar_packet(
                1'000'000 + logical, logical)})));
  }
  testdata::write_mcap(
      dir / "radar.mcap", channel.topic, messages, false);
  const auto catalog =
      scan_mcap_directory_lightweight(dir, {channel});
  CHECK(catalog.required_reorder_frames == 128);
  std::size_t emitted{};
  const auto stats = stream_mcap_frames(
      catalog, {channel}, {.max_reorder_frames = 128},
      [&](const std::string&, joint_sensor_player::LidarFrame) {
        ++emitted;
        return true;
      });
  CHECK(emitted == 128);
  CHECK(stats.max_reorder_frames == 128);
  std::filesystem::remove_all(dir);
}

TEST_CASE(streaming_scan_rejects_129_real_frames_without_parsing_again) {
  const auto dir = fresh_dir("joint_player_mcap_reorder_129");
  const auto channel = default_lidar_channels()[0];
  std::vector<std::vector<std::uint8_t>> messages;
  for (std::uint32_t logical = 129; logical > 0; --logical) {
    messages.push_back(testdata::message(
        1, logical, 100 + logical,
        testdata::udp_frame_packets(
            1, logical * 1000,
            {testdata::main_lidar_packet(
                1'000'000 + logical, logical)})));
  }
  testdata::write_mcap(
      dir / "radar.mcap", channel.topic, messages, false);
  const auto catalog =
      scan_mcap_directory_lightweight(dir, {channel});
  CHECK(catalog.required_reorder_frames == 129);
  bool observer_called = false;
  const auto error = thrown_message([&] {
    (void)stream_mcap_frames(
        catalog, {channel}, {.max_reorder_frames = 128},
        [&](const std::string&, joint_sensor_player::LidarFrame) {
          observer_called = true;
          return true;
        });
  });
  CHECK(!observer_called);
  CHECK(error.find("required=129") != std::string::npos);
  std::filesystem::remove_all(dir);
}

TEST_CASE(lightweight_scan_deduplicates_nonadjacent_source_timestamp) {
  const auto dir = fresh_dir("joint_player_mcap_nonadjacent_duplicate");
  const auto channel = default_lidar_channels()[0];
  const auto first_packet = testdata::main_lidar_packet(1'000'000, 1);
  const auto second_packet = testdata::main_lidar_packet(1'100'000, 2);
  const auto first = testdata::udp_frame_packets(1, 0, {first_packet});
  const auto second =
      testdata::udp_frame_packets(1, 100'000'000, {second_packet});
  testdata::write_mcap(
      dir / "radar.mcap", channel.topic,
      {testdata::message(1, 1, 100, first),
       testdata::message(1, 2, 200, second),
       testdata::message(1, 3, 300, first)},
      false);
  const auto catalog =
      scan_mcap_directory_lightweight(dir, {channel});
  CHECK(catalog.frames.at(channel.topic).size() == 2);
  CHECK(catalog.duplicate_messages == 1);
  std::size_t emitted{};
  (void)stream_mcap_frames(
      catalog, {channel}, {.max_reorder_frames = 3},
      [&](const std::string&, joint_sensor_player::LidarFrame) {
        ++emitted;
        return true;
      });
  CHECK(emitted == 2);
  std::filesystem::remove_all(dir);
}

TEST_CASE(lightweight_scan_rejects_nonadjacent_timestamp_conflict) {
  const auto dir = fresh_dir("joint_player_mcap_nonadjacent_conflict");
  const auto channel = default_lidar_channels()[0];
  const auto first = testdata::udp_frame_packets(
      1, 0, {testdata::main_lidar_packet(1'000'000, 1)});
  const auto second = testdata::udp_frame_packets(
      1, 100'000'000,
      {testdata::main_lidar_packet(1'100'000, 2)});
  const auto conflict = testdata::udp_frame_packets(
      1, 0, {testdata::main_lidar_packet(1'200'000, 3)});
  testdata::write_mcap(
      dir / "radar.mcap", channel.topic,
      {testdata::message(1, 1, 100, first),
       testdata::message(1, 2, 200, second),
       testdata::message(1, 3, 300, conflict)},
      false);
  const auto error = thrown_message([&] {
    (void)scan_mcap_directory_lightweight(dir, {channel});
  });
  CHECK(error.find("同一topic同一时间戳内容冲突") !=
        std::string::npos);
  std::filesystem::remove_all(dir);
}
