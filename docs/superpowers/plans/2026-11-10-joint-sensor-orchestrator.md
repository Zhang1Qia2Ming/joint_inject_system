# Joint Sensor Orchestrator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a one-shot C++ coordinator for eight lidar streams and three camera streams, with bounded UYVY caching, mock/XDMA video backends, lifecycle-based fail-stop behavior, and schema-v2 evidence.

**Architecture:** Add an isolated `joint_sensor_orchestrator_lib` that links the existing lidar library without changing either existing CLI's defaults. A Python cache tool extracts the three H.265 topics and preserves source timestamps; the C++ runtime validates the cache, schedules video and lidar from one monotonic start, and records a unified report.

**Tech Stack:** C++20, CMake, yaml-cpp, nlohmann-json, POSIX sockets/files, Python 3, `mcap`, FFmpeg, existing custom C++ test framework.

## Global Constraints

- Existing `joint_sensor_player` and `hil_lidar_emulator` behavior and tests must remain unchanged.
- First version accepts `loop_count: 1` only.
- Default video backend is `mock`; XDMA requires explicit configuration.
- Default video and lidar lateness limits are 5000 us and 20000 us.
- Video queues contain 3–6 frames, default 5; full UYVY files must never be loaded into RAM.
- TX1/TX4 map 10 Hz source content to three 30 Hz output slots; TX2 maps 30 Hz one-to-one.
- Missing topics, time rollback, cache corruption, queue starvation, short writes, device loss, or lateness trigger global stop.
- Thor video reception, physical lidar-camera synchronization, hardware FSYNC, and dual-card seven-camera validation are recorded as `NOT_RUN`.
- Every behavior change follows red-green TDD.
- This directory is not a Git repository, so commit steps are replaced by file/status review checkpoints.

---

### Task 1: Orchestrator Configuration and Lifecycle

**Files:**
- Create: `include/joint_sensor_orchestrator/orchestrator_config.hpp`
- Create: `src/orchestrator_config.cpp`
- Create: `include/joint_sensor_orchestrator/lifecycle_manager.hpp`
- Create: `src/lifecycle_manager.cpp`
- Create: `tests/test_orchestrator_config.cpp`
- Create: `tests/test_lifecycle_manager.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `OrchestratorConfig load_orchestrator_config(path)`
- Produces: `LifecycleManager::transition(LifecycleState)`, `fail(component, reason)`, `request_stop()`
- Produces: first-failure preservation and a reusable `CancellationToken`

- [ ] **Step 1: Write failing configuration tests**

```cpp
TEST_CASE(orchestrator_config_is_mock_one_shot_and_bounded) {
  const auto config = load_orchestrator_config(path);
  CHECK(config.video_backend == VideoBackend::Mock);
  CHECK(config.loop_count == 1);
  CHECK(config.video_queue_frames == 5);
  CHECK(config.video_max_lateness_us == 5000);
  CHECK(config.lidar_max_lateness_us == 20000);
}

TEST_CASE(orchestrator_rejects_loop_and_unsafe_queue) {
  CHECK_THROWS(load_config_text("playback:\n  loop_count: 0\n"));
  CHECK_THROWS(load_config_text("video:\n  queue_frames: 7\n"));
}
```

- [ ] **Step 2: Run the tests and verify RED**

Run: `cmake --build build -j4`

Expected: compilation fails because orchestrator configuration types do not exist.

- [ ] **Step 3: Implement configuration types and validation**

```cpp
enum class VideoBackend { Mock, Xdma };
struct VideoLaneConfig {
  std::string name;
  std::string topic;
  unsigned h2c{};
  unsigned source_fps{};
  unsigned output_fps{30};
};
struct OrchestratorConfig {
  joint_sensor_player::PlayerConfig lidar;
  VideoBackend video_backend{VideoBackend::Mock};
  std::filesystem::path video_mcap;
  std::filesystem::path cache_dir;
  std::uint32_t video_queue_frames{5};
  std::uint32_t video_max_lateness_us{5000};
  std::uint32_t lidar_max_lateness_us{20000};
  std::uint64_t loop_count{1};
  std::array<VideoLaneConfig, 3> lanes;
};
```

Validation rejects non-one-shot playback, queue sizes outside 3–6, duplicate lanes/H2C indices, rates other than 10→30 or 30→30, and empty paths/topics.

- [ ] **Step 4: Write failing lifecycle tests**

```cpp
TEST_CASE(lifecycle_requires_order_and_preserves_first_failure) {
  LifecycleManager manager;
  manager.transition(LifecycleState::Preparing);
  manager.transition(LifecycleState::Ready);
  CHECK_THROWS(manager.transition(LifecycleState::Running));
  manager.fail("video", "short write");
  manager.fail("radar", "cancelled");
  CHECK(manager.failure().component == "video");
  CHECK(manager.stop_token().stop_requested());
}
```

- [ ] **Step 5: Implement the lifecycle state machine**

Permit only the approved transitions, atomically preserve the first failure, expose snapshots for reports, and share one stop token across engines.

- [ ] **Step 6: Run foundation tests**

Run: `ctest --test-dir build --output-on-failure`

Expected: existing tests and new configuration/lifecycle tests pass.

- [ ] **Step 7: Review checkpoint**

Run: `rg 'loop_count|queue_frames|VideoBackend|LifecycleState' include/joint_sensor_orchestrator src tests`

Expected: one authoritative definition for each contract.

---

### Task 2: Video MCAP Index and Persistent UYVY Cache

**Files:**
- Create: `tools/build_video_cache.py`
- Create: `include/joint_sensor_orchestrator/video_cache.hpp`
- Create: `src/video_cache.cpp`
- Create: `tests/test_video_cache.cpp`
- Create: `tests/test_build_video_cache.py`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Python produces one `.uyvy`, one `.timestamps_ns`, and one `manifest.json` entry per lane.
- C++ produces: `VideoCache load_video_cache(cache_dir, expected_config)`
- Cache frame size is exactly `3840 * 2160 * 2 = 16588800`.

- [ ] **Step 1: Write failing Python tests for timestamp-preserving extraction**

```python
def test_lane_manifest_preserves_message_times(tmp_path):
    manifest = build_fixture_cache(fixture_mcap, tmp_path)
    lane = manifest["lanes"]["tx1"]
    assert lane["frame_bytes"] == 16_588_800
    assert lane["source_timestamps_ns"] == [100_000_000, 200_000_000]
    assert lane["sha256"]
```

- [ ] **Step 2: Verify Python RED**

Run: `python3 -m unittest tools/test_build_video_cache.py -v`

Expected: import or symbol failure because the cache tool does not exist.

- [ ] **Step 3: Implement the cache tool**

Use `mcap.reader.make_reader`, retain `message.log_time` for every Annex-B packet after the first VPS, decode each lane with:

```bash
ffmpeg -v error -i lane.h265 -f rawvideo -pix_fmt uyvy422 pipe:1
```

Write output to temporary files, verify decoded frame count equals retained timestamp count and file size equals `count * 16588800`, calculate SHA-256, then atomically rename into the cache directory. Manifest version is `1` and includes source MCAP size/mtime, converter version, topic, timestamps file, frame count, frame bytes, and checksum.

- [ ] **Step 4: Write failing C++ cache validation tests**

```cpp
TEST_CASE(cache_rejects_truncated_or_changed_payload) {
  write_manifest_and_payload(root, 2, kFrameBytes * 2);
  CHECK(load_video_cache(root, expected).lanes[0].frame_count == 2);
  std::filesystem::resize_file(payload, kFrameBytes * 2 - 1);
  CHECK_THROWS(load_video_cache(root, expected));
}
```

- [ ] **Step 5: Implement C++ manifest validation**

Parse JSON, validate version, topic/rate/format, strictly increasing timestamps, exact file size, frame count, and SHA-256 recorded by the builder. Return memory-light metadata and paths only.

- [ ] **Step 6: Run cache tests**

Run:

```bash
python3 -m unittest tools/test_build_video_cache.py -v
ctest --test-dir build --output-on-failure
```

Expected: all tests pass and no test creates a full-size real frame.

- [ ] **Step 7: Review checkpoint**

Verify failed conversion leaves no valid manifest and no final payload with:

`rg 'rename|manifest|sha256|timestamps_ns' tools/build_video_cache.py src/video_cache.cpp`

---

### Task 3: Unified Timeline, Bounded Queue, and Video Sinks

**Files:**
- Create: `include/joint_sensor_orchestrator/unified_timeline.hpp`
- Create: `src/unified_timeline.cpp`
- Create: `include/joint_sensor_orchestrator/video_frame_queue.hpp`
- Create: `src/video_frame_queue.cpp`
- Create: `include/joint_sensor_orchestrator/video_sink.hpp`
- Create: `src/mock_video_sink.cpp`
- Create: `src/xdma_video_sink.cpp`
- Create: `tests/test_unified_timeline.cpp`
- Create: `tests/test_video_frame_queue.cpp`
- Create: `tests/test_video_sink.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `build_video_slots(source_timestamps, source_fps, common_window)`
- Produces: blocking bounded `VideoFrameQueue` with cancellation-aware push/pop.
- Produces: `VideoSink::prepare`, `write_frame`, `health`, `stop`.

- [ ] **Step 1: Write failing timeline tests**

```cpp
TEST_CASE(ten_hz_content_maps_to_three_distinct_thirty_hz_slots) {
  const auto slots = build_video_slots({1'000'000'000, 1'100'000'000}, 10, 30);
  CHECK(slots.size() == 6);
  CHECK(slots[0].source_index == 0);
  CHECK(slots[1].source_index == 0);
  CHECK(slots[2].source_index == 0);
  CHECK(slots[3].source_index == 1);
  CHECK(slots[1].relative_ns - slots[0].relative_ns == 33'333'333);
}
```

Also test TX4's approximately 100 ms source offset remains visible and common-window clipping never invents an earlier frame.

- [ ] **Step 2: Implement timeline mapping**

Use source timestamps for content identity and deterministic 30 Hz integer deadlines. Reject backward timestamps, unsupported ratios, overflow, and slots outside the common window.

- [ ] **Step 3: Write and implement bounded queue tests**

Test capacity blocking, cancellation wake-up, EOF, and maximum observed waterline. Store frame buffers per slot only; never copy the whole source file.

- [ ] **Step 4: Write failing sink tests**

```cpp
TEST_CASE(mock_sink_rejects_wrong_frame_and_injected_failure) {
  MockVideoSink sink({.frame_bytes = kFrameBytes, .fail_at_frame = 3});
  CHECK(sink.write_frame(valid_frame, slot0).ok);
  CHECK(!sink.write_frame(short_frame, slot1).ok);
}

TEST_CASE(xdma_write_retries_eintr_but_rejects_short_write) {
  CHECK(write_full_frame_with(fake_eintr_then_full, data, 16));
  CHECK(!write_full_frame_with(fake_short_write, data, 16));
}
```

- [ ] **Step 5: Implement Mock and XDMA backends**

Mock validates lane, frame size, source/output sequence, deadline/lateness, and supports deterministic failure injection. XDMA discovers `/dev/xdma*_h2c_{0,1,2}`, verifies the selected device exposes all lanes, uses `O_WRONLY`, loops only for `EINTR`, and treats any positive short write as fatal.

- [ ] **Step 6: Run runtime primitive tests**

Run: `ctest --test-dir build --output-on-failure`

Expected: timeline, queue, and sink tests pass with no XDMA hardware access in default tests.

---

### Task 4: Explicit Radar Start and Engine Adapters

**Files:**
- Modify: `include/joint_sensor_player/player.hpp`
- Modify: `src/player.cpp`
- Modify: `tests/test_player.cpp`
- Create: `include/joint_sensor_orchestrator/radar_engine.hpp`
- Create: `src/radar_engine.cpp`
- Create: `include/joint_sensor_orchestrator/video_engine.hpp`
- Create: `src/video_engine.cpp`
- Create: `tests/test_radar_engine.cpp`
- Create: `tests/test_video_engine.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Extends: `play_catalog(..., const LidarPacketRewriter*, std::optional<uint64_t> explicit_play_start_ns)`
- Produces: `RadarEngine::prepare/arm/run/stop/result`
- Produces: `VideoEngine::prepare/arm/run/stop/result`

- [ ] **Step 1: Write failing explicit-start regression test**

```cpp
TEST_CASE(player_uses_explicit_shared_start_when_supplied) {
  FakeClock clock(500);
  const auto result = play_catalog(config, catalog, clock, sink, nullptr,
                                   nullptr, nullptr, 10'000);
  CHECK(result.play_start_ns == 10'000);
  CHECK(sink.deadlines.front() == 10'000);
}
```

- [ ] **Step 2: Implement optional explicit start**

Existing callers omit the final argument and preserve `now + start_delay`. Reject an explicit start earlier than current monotonic time. Do not change loop, cancellation, or lateness defaults.

- [ ] **Step 3: Write failing RadarEngine tests**

Use fake clock/sink and verify it remains READY until armed, uses the shared start and UTC anchor, propagates send failure to `LifecycleManager`, and stops on shared cancellation.

- [ ] **Step 4: Implement RadarEngine**

Build the radar catalog/window, clip it to the orchestrator's common source window, build one-shot `LidarPacketRewriter`, optionally start PTC, and call `play_catalog` in its worker thread with the explicit shared monotonic start.

- [ ] **Step 5: Write failing VideoEngine tests**

Verify source frames are read by `pread` into at most queue capacity buffers, TX1/TX4 repeat slots reuse source identity, TX2 remains one-to-one, and sink failure cancels all engines.

- [ ] **Step 6: Implement VideoEngine**

Each lane owns one producer and one consumer. Producer uses `pread` by source index; consumer waits for absolute slot deadlines, applies the 5 ms lateness limit, writes to the configured sink, and records queue waterline and frame statistics.

- [ ] **Step 7: Run engine and regression tests**

Run: `ctest --test-dir build --output-on-failure`

Expected: old player/emulator tests and all engine tests pass.

---

### Task 5: Orchestrator, Schema-v2 Report, and CLI

**Files:**
- Create: `include/joint_sensor_orchestrator/report_v2.hpp`
- Create: `src/report_v2.cpp`
- Create: `include/joint_sensor_orchestrator/orchestrator.hpp`
- Create: `src/orchestrator.cpp`
- Create: `src/orchestrator_main.cpp`
- Create: `tests/test_report_v2.cpp`
- Create: `tests/test_orchestrator_integration.cpp`
- Create: `config/orchestrator_mock.yaml`
- Modify: `CMakeLists.txt`
- Modify: `README.md`

**Interfaces:**
- Produces executable: `joint_sensor_orchestrator --config FILE [--prepare-cache] [--dry-run]`
- Produces schema-v2 `summary.json`, `video_frames.csv`, existing-compatible radar frame CSV, anomalies, and run log.

- [ ] **Step 1: Write failing report tests**

```cpp
TEST_CASE(report_v2_marks_unexecuted_hardware_layers_not_run) {
  const auto json = read_summary(run_dir);
  CHECK(json["schema_version"] == 2);
  CHECK(json["validation"]["software_logic"] == "PASS");
  CHECK(json["validation"]["thor_nvsipl"] == "NOT_RUN");
  CHECK(json["validation"]["physical_sync"] == "NOT_RUN");
  CHECK(json["validation"]["hardware_fsync"] == "NOT_RUN");
}
```

- [ ] **Step 2: Implement report v2 without changing v1**

Create a separate report implementation. Record lifecycle transitions, common window, anchors, per-lane source/output counts, queue waterlines, lateness distribution, backend, first failure, and validation layers. Never infer hardware PASS from Mock execution.

- [ ] **Step 3: Write failing orchestrator integration tests**

Cover all-ready barrier, preparation failure, runtime video failure, runtime radar failure, Ctrl-C cancellation, and successful one-shot convergence to STOPPED.

- [ ] **Step 4: Implement orchestration**

Prepare catalog/cache/engines, transition lifecycle, set:

```cpp
shared_play_start_ns = monotonic_now_ns + start_delay_ns;
utc_anchor_us = realtime_now_us + start_delay_ms * 1000;
```

Arm both engines, release one barrier, wait for both, request global stop on first failure, join every worker, stop PTC/sinks, and finish the report exactly once.

- [ ] **Step 5: Implement CLI and safe configuration**

`--prepare-cache` invokes `tools/build_video_cache.py` with an argument vector and fails if the subprocess or manifest validation fails. `--dry-run` scans and validates without sending. The default YAML uses Mock and `loop_count: 1`.

- [ ] **Step 6: Wire CMake**

Add `joint_sensor_orchestrator_lib`, `joint_sensor_orchestrator`, a separate `joint_sensor_orchestrator_tests`, `orchestrator_cli_help`, and Python cache tests. Link the new library publicly to `joint_sensor_player_lib`; do not add video sources to the old library.

- [ ] **Step 7: Run all synthetic tests**

Run:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

Expected: zero failures, including unchanged old CLI tests.

---

### Task 6: Real 40-Second Cache and Mock Joint Playback

**Files:**
- Generated: `/data/joint_sensor_player/cache/1_00062_20261021-101724/**`
- Generated: `reports/orchestrator-*/**`
- Modify only if evidence exposes a defect: files owned by Tasks 2–5, with a new failing regression test first.

**Interfaces:**
- Consumes:
  - Video MCAP: `.../1_00062_20261021-101724-D/1_00062_20261021-101724-D_0.mcap`
  - Lidar MCAP: `.../1_00062_20261021-101724-B-SENSOR/1_00062_20261021-101724-B-SENSOR_0.mcap`

- [ ] **Step 1: Confirm prerequisites and free space**

Run:

```bash
python3 -c 'import mcap'
ffmpeg -version
df -h /data
```

Expected: Python import and FFmpeg succeed; `/data` has at least 40 GiB free.

- [ ] **Step 2: Build the complete cache**

Run:

```bash
./build/joint_sensor_orchestrator \
  --config config/orchestrator_mock.yaml \
  --prepare-cache
```

Expected: three atomic payloads, timestamps, and a valid manifest; expected source counts are approximately 401/1200/399, with exact values governed by VPS-gated extraction and strict timestamp/frame equality.

- [ ] **Step 3: Verify cache and bounded memory**

Run the orchestrator dry-run and inspect summary/cache sizes. During playback, sample process RSS and assert it stays below 1 GiB, proving the 31 GiB cache is not preloaded.

- [ ] **Step 4: Run real-time one-shot Mock playback**

Stop any competing lidar emulator normally, run:

```bash
./build/joint_sensor_orchestrator \
  --config config/orchestrator_mock.yaml
```

Expected: approximately 40 seconds wall time, eight lidar streams complete, three video lanes complete once, no independent rewind, no lateness/failure, lifecycle ends STOPPED.

- [ ] **Step 5: Validate evidence**

Check:

- schema version 2;
- software logic `PASS`;
- XDMA/Thor/NvSIPL/physical sync/FSYNC `NOT_RUN`;
- TX1/TX4 output count is three times retained source content after common-window clipping;
- TX2 output count is one-to-one;
- queue maximum never exceeds configured five;
- no source timestamp rollback;
- all existing lidar tests remain green.

- [ ] **Step 6: Run the full final matrix**

Run:

```bash
ctest --test-dir build --output-on-failure
cmake --build build-release -j"$(nproc)"
ctest --test-dir build-release --output-on-failure
```

Expected: zero test failures in Debug/RelWithDebInfo and Release.

- [ ] **Step 7: Final review checkpoint**

Read the latest report and compare every claim against evidence. Report hardware-dependent rows only as `NOT_RUN`; do not convert them to PASS or FAIL without physical hardware execution.
