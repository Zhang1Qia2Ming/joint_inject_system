#include "joint_sensor_orchestrator/unified_timeline.hpp"
#include "test_framework.hpp"

#include <cstdint>
#include <limits>
#include <vector>

using joint_sensor_orchestrator::SourceWindow;
using joint_sensor_orchestrator::build_video_slots;

TEST_CASE(ten_hz_content_maps_to_three_slots_and_preserves_source_time) {
  const std::vector<std::uint64_t> timestamps{
      1'000'000'000, 1'100'000'000};
  const auto slots = build_video_slots(
      timestamps, 10, 30, SourceWindow{1'000'000'000, 1'199'999'999},
      5'000'000'000);

  CHECK(slots.size() == 6);
  CHECK(slots[0].source_index == 0);
  CHECK(slots[1].source_index == 0);
  CHECK(slots[2].source_index == 0);
  CHECK(slots[3].source_index == 1);
  CHECK(slots[0].source_time_ns == 1'000'000'000);
  CHECK(slots[2].source_time_ns == 1'000'000'000);
  CHECK(slots[0].grid_index == 0);
  CHECK(slots[3].grid_index == 3);
  CHECK(slots[1].slot_time_ns - slots[0].slot_time_ns == 33'333'333);
  CHECK(slots[3].relative_ns == 100'000'000);
  CHECK(slots[3].deadline_ns == 5'100'000'000);
}

TEST_CASE(thirty_hz_mapping_preserves_lane_phase) {
  const std::vector<std::uint64_t> timestamps{
      1'084'000'000, 1'117'000'000};
  const auto slots = build_video_slots(
      timestamps, 30, 30, SourceWindow{1'000'000'000, 1'200'000'000},
      9'000'000'000);

  CHECK(slots.size() == 2);
  CHECK(slots[0].source_time_ns == 1'084'000'000);
  CHECK(slots[0].slot_time_ns == 1'100'000'000);
  CHECK(slots[0].relative_ns == 100'000'000);
  CHECK(slots[1].relative_ns == 133'333'333);
  CHECK(slots[0].grid_index == 3);
  CHECK(slots[1].grid_index == 4);
}

TEST_CASE(ten_hz_mapping_preserves_tx4_phase_against_common_start) {
  const auto slots = build_video_slots(
      {1'100'000'000}, 10, 30,
      SourceWindow{1'000'000'000, 1'200'000'000}, 2'000'000'000);

  CHECK(slots.size() == 3);
  CHECK(slots[0].source_time_ns == 1'100'000'000);
  CHECK(slots[0].grid_index == 3);
  CHECK(slots[0].relative_ns == 100'000'000);
  CHECK(slots[1].relative_ns == 133'333'333);
  CHECK(slots[2].relative_ns == 166'666'666);
}

TEST_CASE(ten_hz_tail_with_only_one_or_two_slots_is_excluded) {
  CHECK(build_video_slots(
            {1'000'000'000}, 10, 30,
            SourceWindow{1'000'000'000, 1'000'000'000},
            4'000'000'000)
            .empty());
  CHECK(build_video_slots(
            {1'000'000'000}, 10, 30,
            SourceWindow{1'000'000'000, 1'033'333'333},
            4'000'000'000)
            .empty());
}

TEST_CASE(ten_hz_tail_is_included_when_third_slot_hits_window_end) {
  const auto slots = build_video_slots(
      {1'000'000'000}, 10, 30,
      SourceWindow{1'000'000'000, 1'066'666'666},
      4'000'000'000);
  CHECK(slots.size() == 3);
  CHECK(slots[0].slot_time_ns == 1'000'000'000);
  CHECK(slots[2].slot_time_ns == 1'066'666'666);
}

TEST_CASE(thirty_hz_tail_remains_one_source_to_one_slot) {
  const auto slots = build_video_slots(
      {1'000'000'000, 1'033'333'333, 1'066'666'666}, 30, 30,
      SourceWindow{1'000'000'000, 1'033'333'333},
      4'000'000'000);
  CHECK(slots.size() == 2);
  CHECK(slots[0].source_index == 0);
  CHECK(slots[1].source_index == 1);
}

TEST_CASE(timeline_rejects_bad_rates_order_window_and_overflow) {
  CHECK_THROWS(build_video_slots(
      {100, 99}, 10, 30, SourceWindow{0, 1'000}, 0));
  CHECK_THROWS(build_video_slots(
      {100}, 20, 30, SourceWindow{0, 1'000}, 0));
  CHECK_THROWS(build_video_slots(
      {100}, 10, 30, SourceWindow{200, 100}, 0));
  CHECK_THROWS(build_video_slots(
      {std::numeric_limits<std::uint64_t>::max() - 1}, 10, 30,
      SourceWindow{0, std::numeric_limits<std::uint64_t>::max()}, 0));
  CHECK_THROWS(build_video_slots(
      {1'000'000'000, 1'100'000'000}, 10, 30,
      SourceWindow{1'000'000'000, 1'200'000'000},
      std::numeric_limits<std::uint64_t>::max() - 1));
}

TEST_CASE(timeline_resolves_nearest_slot_collisions_strictly_forward) {
  const auto slots = build_video_slots(
      {1'010'000'000, 1'011'000'000}, 30, 30,
      SourceWindow{1'000'000'000, 1'100'000'000}, 2'000'000'000);
  CHECK(slots.size() == 2);
  CHECK(slots[0].grid_index == 0);
  CHECK(slots[1].grid_index == 1);
}

TEST_CASE(timeline_integer_grid_has_no_long_term_cumulative_error) {
  constexpr std::uint64_t kSourceStart = 10'000'000'000;
  constexpr std::uint64_t kPlayStart = 20'000'000'000;
  constexpr std::uint64_t kLastIndex = 108'000;
  std::vector<std::uint64_t> timestamps;
  timestamps.reserve(kLastIndex + 1);
  for (std::uint64_t index = 0; index <= kLastIndex; ++index) {
    timestamps.push_back(
        kSourceStart + (index / 30) * 1'000'000'000ULL +
        ((index % 30) * 1'000'000'000ULL) / 30);
  }
  const auto slots = build_video_slots(
      timestamps, 30, 30,
      SourceWindow{kSourceStart, kSourceStart + 3'600'000'000'000ULL},
      kPlayStart);

  CHECK(slots.size() == timestamps.size());
  CHECK(slots.back().grid_index == kLastIndex);
  CHECK(slots.back().relative_ns == 3'600'000'000'000ULL);
  CHECK(slots.back().deadline_ns ==
        kPlayStart + 3'600'000'000'000ULL);
}
