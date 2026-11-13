# Eight-Lidar Player Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a C++20 player that discovers eight lidar topics from MCAP files, preserves their recorded timing, and sends raw UDP to ThorA over one 1 Gbps interface.

**Architecture:** A read-only indexing stage merges MCAP segments by source timestamp and fills bounded per-lidar frame queues. A single monotonic-clock scheduler releases frames on absolute deadlines; each frame's packets use the configured intra-frame interval and a source-IP-bound UDP socket. Reporting runs behind a bounded asynchronous queue so disk I/O never blocks playback.

**Tech Stack:** C++20, CMake, yaml-cpp, zstd, POSIX sockets, CLOCK_MONOTONIC, nlohmann/json, CTest.

## Global Constraints

- Phase one enables eight lidar channels only; camera interfaces remain an extension boundary.
- Default input directory is `/data/joint_sensor_player/data`; only first-level `.mcap` files are scanned.
- Playback is one-shot at strict 1x speed with no pause, loop, or runtime mode switching.
- Raw lidar UDP payloads and embedded timestamps are not rewritten.
- All eight channels use `enp3s0f0`, source IPs `192.168.1.31` through `.38`, source port `10000`, and ThorA destination `192.168.1.4`.
- Main lidar packets use 83,200 ns spacing; short lidar packets use 193,300 ns spacing.
- `allow_partial_start` defaults to false; all enabled channels must be ready before a common start.
- Timing takes priority over reporting. Normal report records may be dropped under pressure, but playback may not block.
- Large MCAP files remain under `/data`; source, configuration, tests, and reports remain under `~/zhang/joint_sensor_player`.
- Existing `mock_lidar` and video-injection projects are read-only references.

---



### Task 1: Project Skeleton, Slot Profiles, and Configuration

**Files:**

- Create: `CMakeLists.txt`
- Create: `include/joint_sensor_player/config.hpp`
- Create: `src/config.cpp`
- Create: `tests/test_config.cpp`
- Create: `config/example.yaml`

**Interfaces:**

- Produces: `PlayerConfig load_config(const std::filesystem::path&)`
- Produces: `std::array<LidarChannelConfig, 8> default_lidar_channels()`

- [ ] Write tests for the eight fixed slot profiles, default strict-start semantics, YAML overrides, and invalid duplicate topics/ports.
- [ ] Configure CTest and run the test executable; verify compilation fails because the configuration API does not exist.
- [ ] Implement the minimal typed configuration model and yaml-cpp loader.
- [ ] Run CTest and verify all configuration tests pass.



### Task 2: MCAP Discovery, Segment Merge, and UdpFrame Parsing

**Files:**

- Create: `include/joint_sensor_player/mcap.hpp`
- Create: `src/mcap.cpp`
- Create: `tests/test_mcap.cpp`
- Create: `tests/test_data_builder.hpp`

**Interfaces:**

- Consumes: `LidarChannelConfig`
- Produces: `McapCatalog scan_mcap_directory(path, channels)`
- Produces: `std::vector<LidarFrame> read_lidar_frames(catalog, channel)`

- [ ] Write synthetic MCAP fixtures in the test process for uncompressed and zstd chunks containing Fixed626 and Fixed512 messages.
- [ ] Test first-level-only scanning, topic discovery, timestamp ordering across segments, exact duplicate removal, conflicting duplicate rejection, and gap/overlap annotations.
- [ ] Run the MCAP tests and verify they fail because the reader API is missing.
- [ ] Implement bounded MCAP record parsing, zstd decompression, CDR UdpFrame decoding, and deterministic timestamp merge.
- [ ] Run CTest and verify MCAP and configuration tests pass.



### Task 3: Timeline, Packet Deadlines, and Readiness Gate

**Files:**

- Create: `include/joint_sensor_player/timeline.hpp`
- Create: `src/timeline.cpp`
- Create: `tests/test_timeline.cpp`

**Interfaces:**

- Consumes: ordered `LidarFrame` sequences
- Produces: `PlaybackWindow`, frame deadlines, packet deadlines, and readiness diagnostics

- [ ] Write tests for common-intersection selection, strict all-channel readiness, partial-start mode, 1x timestamp mapping, and 83.2/193.3 microsecond packet deadlines.
- [ ] Run the tests and verify expected failures.
- [ ] Implement pure timeline calculations independent of sockets and wall clock.
- [ ] Run CTest and verify all timeline tests pass.



### Task 4: UDP Transport and Absolute-Deadline Scheduler

**Files:**

- Create: `include/joint_sensor_player/udp_sender.hpp`
- Create: `src/udp_sender.cpp`
- Create: `include/joint_sensor_player/player.hpp`
- Create: `src/player.cpp`
- Create: `tests/test_player.cpp`

**Interfaces:**

- Consumes: `PlayerConfig`, `PlaybackWindow`, and per-channel frame streams
- Produces: `PlaybackResult run_player(...)`
- Supports injected clock and datagram sink in tests

- [ ] Write tests using an in-memory clock and datagram sink to prove ordering, source-channel isolation, common start, packet spacing, stop-all behavior, and cancellation on send failure.
- [ ] Run the tests and verify expected failures.
- [ ] Implement one socket per source IP, absolute `clock_nanosleep` scheduling, bounded prefetch queues, and coordinated cancellation.
- [ ] Run CTest and verify all scheduler tests pass without real network traffic.



### Task 5: Non-Blocking Reports

**Files:**

- Create: `include/joint_sensor_player/report.hpp`
- Create: `src/report.cpp`
- Create: `tests/test_report.cpp`

**Interfaces:**

- Produces: `summary.json`, `frame_stats.csv`, `anomalies.jsonl`, and `run.log`
- Produces: a bounded `ReportSink::try_submit` that never waits

- [ ] Write tests for stable report fields, per-frame aggregation, anomaly records, segment annotations, and report-queue overflow counters.
- [ ] Run tests and verify expected failures.
- [ ] Implement the asynchronous report worker and atomic final-file replacement.
- [ ] Run CTest and verify all report tests pass.



### Task 6: CLI, Dry Run, and Network Preparation

**Files:**

- Create: `src/main.cpp`
- Create: `scripts/prepare_network.sh`
- Create: `README.md`

**Interfaces:**

- Produces: `joint_sensor_player --config FILE [--dry-run]`
- Produces: `prepare_network.sh --config FILE [--check|--restore]`

- [ ] Add CLI integration tests for help, invalid configuration, missing topics, dry-run success, and refusal when strict readiness fails.
- [ ] Run tests and verify expected failures.
- [ ] Implement first-level MCAP discovery, preflight output, 1 Gbps/full-duplex checks, domain reachability checks, source-IP setup/restore, and normal playback entry.
- [ ] Run the complete CTest suite.



### Task 7: Local Verification

**Files:**

- Modify only files above if verification exposes a tested defect.

- [ ] Configure a clean build with `cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo`.
- [ ] Build with `cmake --build build -j`.
- [ ] Run `ctest --test-dir build --output-on-failure`.
- [ ] Run `joint_sensor_player --config config/example.yaml --dry-run` against the configured data directory.
- [ ] Run network check mode and confirm `enp3s0f0` is 1000 Mbps full duplex and `192.168.1.4` is reachable.
- [ ] Review generated report schema and document any validation blocked by absent MCAP files or domain-side collector.