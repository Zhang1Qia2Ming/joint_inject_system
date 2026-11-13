# Lidar Loop Playback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add finite and infinite seamless loop playback with optional cross-loop UDP sequence continuity.

**Architecture:** Extend the existing heap scheduler with a virtual cycle offset instead of restarting playback. Add protocol-specific sequence adapters that validate all packets before playback and mutate only the four-byte in-memory sequence field on later cycles. Use an atomic cancellation token for signals and preserve the existing fail-all behavior.

**Tech Stack:** C++20, CMake, CTest, POSIX signals, existing MCAP/UDP/report modules.

## Global Constraints

- Default remains one-shot (`loop_count: 1`); zero means infinite.
- No extra delay is inserted between cycles.
- MCAP files and packet time fields remain unchanged.
- `continuous` and `preserve` sequence modes are configurable.
- Main lidar UDP 4.3 and short lidar UDP 7.3 are the only accepted continuous-mode formats.
- Send failure, >20 ms lateness, sequence validation failure, or arithmetic overflow stops all channels.
- No per-cycle report is added.

---

### Task 1: Loop Configuration

**Files:**
- Modify: `include/joint_sensor_player/config.hpp`
- Modify: `src/config.cpp`
- Modify: `config/example.yaml`
- Modify: `tests/test_config.cpp`

- [ ] Add failing tests for defaults, finite/infinite values, and invalid sequence mode.
- [ ] Run tests and confirm the expected failure.
- [ ] Add `loop_count` and `SequenceMode` parsing/validation.
- [ ] Run tests and confirm they pass.

### Task 2: Protocol Sequence Adapters

**Files:**
- Create: `include/joint_sensor_player/packet_sequence.hpp`
- Create: `src/packet_sequence.cpp`
- Create: `tests/test_packet_sequence.cpp`

- [ ] Add failing tests for AT128 UDP 4.3 offset calculation, short UDP 7.3 offset 530, unknown packets, continuity, loop delta, preserve mode, and uint32 wrap.
- [ ] Run tests and confirm the expected failure.
- [ ] Implement packet recognition, per-channel validation, and in-place per-cycle adjustment.
- [ ] Run all tests and confirm they pass.

### Task 3: Virtual Loop Timeline and Cancellation

**Files:**
- Modify: `include/joint_sensor_player/timeline.hpp`
- Modify: `src/timeline.cpp`
- Modify: `include/joint_sensor_player/player.hpp`
- Modify: `src/player.cpp`
- Modify: `tests/test_timeline.cpp`
- Modify: `tests/test_player.cpp`

- [ ] Add failing tests for median 10 Hz period, two-cycle deadlines, default one-shot behavior, infinite cancellation, and failure stopping all loops.
- [ ] Run tests and confirm the expected failure.
- [ ] Add cycle-period calculation, cycle-aware cursors, cancellation checks, and overflow guards.
- [ ] Run all tests and confirm they pass.

### Task 4: CLI, Signals, Reports, and Documentation

**Files:**
- Modify: `src/main.cpp`
- Modify: `include/joint_sensor_player/report.hpp`
- Modify: `src/report.cpp`
- Modify: `README.md`

- [ ] Add cumulative `loops_completed` and `stop_reason` report fields.
- [ ] Install SIGINT/SIGTERM handlers backed by the cancellation token.
- [ ] Print loop configuration during dry-run and startup.
- [ ] Document finite and infinite pressure-test commands.

### Task 5: Verification

- [ ] Run Debug build and all CTests.
- [ ] Run Release build and all CTests.
- [ ] Run real-MCAP dry-run with `continuous` sequence validation.
- [ ] Confirm current network gate behavior remains unchanged.
