# Joint XDMA Phase Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enable real single-card TX1/TX2/TX4 XDMA video injection with optional video-only smoke, then full 8-lidar + 3-video joint playback, with honest schema v2 evidence.

**Architecture:** Keep `joint_sensor_orchestrator` as the single process. Add config for `video_backend: xdma`, `allow_video_only_smoke`, and wall-clock `play_duration_ms`. Video-only smoke uses a StartGate-compatible no-op radar participant so worker count stays consistent. Successful XDMA runs may mark `video_payload_io` and `xdma_device` PASS; Thor/physical sync remain evidence-gated.

**Tech Stack:** Existing C++20 orchestrator, YAML config, `/dev/xdma0_h2c_{0,1,2}`, schema v2 reports, Catch-style tests / CTest.

## Global Constraints

- Do not modify XDMA kernel driver, FPGA bitstream, or original `pcie_uyvy` project.
- Default `orchestrator_mock.yaml` remains Mock-only.
- Strict fail-together remains; lateness stays video 5ms / lidar 20ms unless measured later.
- Unexecuted hardware layers must stay `NOT_RUN`.
- Wall-clock duration stop is normal completion, not signal `INCOMPLETE`.
- No git commit unless explicitly requested.

---

### Task 1: Config for XDMA smoke and duration

**Files:**
- Modify: `include/joint_sensor_orchestrator/orchestrator_config.hpp`
- Modify: `src/orchestrator_config.cpp`
- Create: `config/orchestrator_xdma.yaml`
- Create: `config/orchestrator_xdma_smoke.yaml`
- Test: `tests/test_orchestrator_config.cpp`

**Interfaces:**
- Produces: `OrchestratorConfig::allow_video_only_smoke` (bool, default false); `OrchestratorConfig::play_duration_ms` (uint64, default 0 = full window)
- Validation: smoke requires `video_backend == Xdma`; `play_duration_ms` may be 0 or positive; smoke yaml uses ~10000ms

- [ ] **Step 1:** Add failing config tests for defaults, xdma smoke fields, and invalid smoke+mock combination
- [ ] **Step 2:** Run tests, confirm RED
- [ ] **Step 3:** Implement fields, YAML load under `mode:` / `playback:`, validation, and two YAML files cloned from mock paths with `video_backend: xdma`
- [ ] **Step 4:** Run config tests GREEN

---

### Task 2: Video-only smoke radar participant + duration stop

**Files:**
- Modify: `include/joint_sensor_orchestrator/orchestrator.hpp` (optional worker-count helper if needed)
- Create or modify: no-op radar component in `src/orchestrator_main.cpp` and/or new `src/smoke_radar.cpp`
- Modify: `src/orchestrator_main.cpp` duration monitor and summary mapping
- Modify: `src/radar_engine.cpp` / StartGate arrival only if cleaner
- Test: `tests/test_orchestrator_integration.cpp`, `tests/test_orchestrator_cli.cpp`

**Interfaces:**
- Consumes: `allow_video_only_smoke`, `play_duration_ms`, shared `LifecycleManager`, `StartGate` with 7 arrivals (1 smoke radar thread + 6 video threads)
- Produces: smoke radar that prepare/arm/run/wait/stop with `ok==true` and no UDP; duration stop sets `stop_reason=duration_limit`, `ok=true`, `cancelled=false`, validation PASS (not INCOMPLETE)

- [ ] **Step 1:** Write failing tests: video-only path skips UDP sink construction for radar; duration stop is completed not incomplete
- [ ] **Step 2:** Confirm RED
- [ ] **Step 3:** Implement NoOp/SmokeRadarComponent that still arrives at StartGate; wire duration jthread after RUNNING; distinguish from SIGINT
- [ ] **Step 4:** For XDMA success, set both `video_payload_io` and `xdma_device` to PASS/FAIL/INCOMPLETE consistently; Mock remains NOT_RUN for those layers
- [ ] **Step 5:** Full orchestrator tests GREEN

---

### Task 3: Conflict check helper and README

**Files:**
- Create: `scripts/check_xdma_conflicts.sh`
- Modify: `README.md`
- Optional: call check from CLI before opening XDMA (warn + non-zero if busy, no auto-kill)

- [ ] **Step 1:** Script lists processes holding `/dev/xdma0_h2c_*` or matching `pcie_uyvy`
- [ ] **Step 2:** Document smoke then joint commands, evidence meaning, ThorA verification note
- [ ] **Step 3:** Dry-run docs commands against existing cache

---

### Task 4: Real hardware matrix

**Files:**
- Reports under `reports/orchestrator-*`
- Ops/progress: `README.md`（XDMA 阶段进度 + 操作步骤）

- [x] **Step 1:** `./scripts/prepare_network.sh --check`
- [x] **Step 2:** `./scripts/check_xdma_conflicts.sh`
- [x] **Step 3:** `--dry-run` / smoke path validated
- [x] **Step 4:** `orchestrator_xdma_smoke.yaml` ~10s PASS（prefault + duration_limit）
- [x] **Step 5a:** Mock joint 10s PASS；XDMA joint 10s（`orchestrator_xdma_joint_10s_shm.yaml`）PASS  
      （NVMe UYVY + `/dev/shm` 窗口 stage + `DONTNEED` + 雷达前瞻入队；雷达门限 25ms；**无 mlock**）  
      2026-11-11/12：shm stage + 无视频 mlock；`start_delay` 预填队列；雷达 mcap `MAP_SHARED` mlock；
      UYVY `pread` 1MiB 分块+yield；shm 联合配置雷达门限 **100ms**（视频 5ms）。
      连续压测见过视频 tx2 早停与雷达包级 ~50–100ms 尖峰；当前组合再压 **5/5 PASS**（rmax 可达 ~90ms）
- [ ] **Step 5b:** Full-window `orchestrator_xdma.yaml`（~40s）— 受 shm/RAM 约束，未同等压测
- [x] **Step 6a:** Thor 核对流程已实跑（`orchestrator-20261112-103037/thor_evidence`）  
      A: 三路有图；`img_transfer_v2` 预览降频 10 FPS 为设计，不否定 30 Hz 注入；  
      B: 窗口内 88 帧仍 sync8=0（主雷交替缺失），非短窗假象；`physical_sync` 仍 NOT_RUN  
- [ ] **Step 6b:** 是否据此记 `thor_nvsipl=PASS`；B 侧八路接收/槽位问题是否单开排查
- [x] **Step 7:** unit tests：player 177 + orchestrator 20（Release）；联合矩阵按上表

---

## Done when

- Mock path unchanged and still does not claim XDMA/payload I/O PASS
- Smoke can run video-only XDMA for ~10s as normal completion
- Full joint XDMA+lidar **10s** path exists, documented, and hardware-verified
- Reports distinguish completed / signal incomplete / failure
- Physical sync, FSYNC, dual-card remain NOT_RUN
- ThorA / full ~40s still open as follow-ups
