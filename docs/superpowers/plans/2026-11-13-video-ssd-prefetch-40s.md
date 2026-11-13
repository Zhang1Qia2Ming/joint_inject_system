# 三路视频 40 秒 SSD 预读 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用现有一张注入卡，把前远、前广、后视从已在 SSD 上的 UYVY 缓存播完整段（约 40 秒），内存里只保留大约 5 秒的滑动预读。

**Architecture:** `video_prefetch_ms == 0` 时保持今天的整段 `/dev/shm` stage，10 秒配置不变。`video_prefetch_ms > 0` 时不把播放窗口拷进 `/dev/shm`。新增 `VideoAheadCache`：后台按 1 MiB 分块从 SSD 把未来 5 秒的不重复源帧读进 RAM；播放线程只从这块 RAM 取帧，输出队列仍是 5 帧。点云继续用 `/dev/shm/joint_radar_data`。

**Tech Stack:** 现有 C++20 orchestrator、YAML、Catch 风格 `TEST_CASE`、Release 二进制 `build-release/joint_sensor_orchestrator`。

## Global Constraints

- 视频门限保持 `video_max_lateness_us: 5000`。
- 雷达门限保持 `lidar_max_lateness_us: 100000`。
- `start_delay_ms` 保持 5000，既是回车后的预热，也是预读深度。
- 只播现有三路：`tx1` 前远 10→30 Hz，`tx2` 前广 30 Hz，`tx4` 后视 10→30 Hz。四路周视不解码、不发送。
- 不改 `lidar_service`。不把 `physical_sync` 标成 PASS。
- 不把整段 40 秒拷进 `/dev/shm` 或注入卡。
- 不主动 `git commit`，除非用户另说。
- 点云数据目录仍是 `/dev/shm/joint_radar_data`（整段约 1.1 GiB 已经在里面）。

---

### Task 1: 配置项 `video_prefetch_ms`

**Files:**
- Modify: `joint_sensor_player/include/joint_sensor_orchestrator/orchestrator_config.hpp`
- Modify: `joint_sensor_player/src/orchestrator_config.cpp`
- Test: `joint_sensor_player/tests/test_orchestrator_config.cpp`

**Interfaces:**
- Produces: `OrchestratorConfig::video_prefetch_ms`，`std::uint32_t`，默认 `0`。YAML 键 `playback.video_prefetch_ms`。

- [ ] **Step 1: 写失败测试**

在 `test_orchestrator_config.cpp` 增加：缺省为 0；YAML 写成 5000 时读回 5000。

- [ ] **Step 2: 跑测试，确认失败**

`video_prefetch_ms` 尚不存在，编译失败。

- [ ] **Step 3: 实现**

`OrchestratorConfig` 增加 `std::uint32_t video_prefetch_ms{0};`。`load_orchestrator_config` 在 `playback` 节点里 `set_if_present(playback, "video_prefetch_ms", config.video_prefetch_ms)`。

- [ ] **Step 4: 测试通过**

只跑 `test_orchestrator_config`。

### Task 2: 5 秒滑动预读，不拷整段

**Files:**
- Create: `joint_sensor_player/include/joint_sensor_orchestrator/video_ahead_cache.hpp`
- Create: `joint_sensor_player/src/video_ahead_cache.cpp`
- Modify: `joint_sensor_player/src/video_engine.cpp`（`producer_main` 里 XDMA 的 pread 分支）
- Modify: `joint_sensor_player/include/joint_sensor_orchestrator/video_engine.hpp`（依赖里加可选 ahead cache）
- Modify: `joint_sensor_player/src/orchestrator_main.cpp`（约 493–509 行的 stage 分支）
- Test: `joint_sensor_player/tests/test_video_ahead_cache.cpp`
- Modify: `joint_sensor_player` 的 CMake，把新源文件和测试编进去

**Interfaces:**
- Consumes: `VideoCache` 的 `payload_path`、`frame_bytes`、`timestamps_ns`；`video_prefetch_ms`。
- Produces:
  - `VideoAheadCache::VideoAheadCache(const VideoCache&, std::uint64_t prefetch_ns, VideoPreadFunction)`
  - `std::shared_ptr<const VideoFrameBuffer> acquire(lane, source_index, stop_token)`：帧已在 RAM 则直接返回；否则等到该帧被读入或停止。
  - `void release_before(lane, source_index)`：丢掉已经交给输出队列的更早源帧。输出队列里的 `shared_ptr` 仍持有正在发送的那一帧。
  - 读盘仍走现有 `xdma_pread_mutex_`，1 MiB 一块，块间 `yield`。40 秒模式不要再对整文件 `DONTNEED`。

行为：

- 预读窗口是源时间轴上的 5 秒，不是 5 帧。前广约 150 帧，前远和后视各约 50 帧。三路合计约 4.7 GiB。这些缓冲不做 `mlock`。
- 填充线程在 `StartGate` 放开后、`play_start` 之前的 `start_delay` 里把第一段 5 秒读满。播放中保持「当前源时间 + 5 秒」都在 RAM 里。
- 播放线程原来的队列深度仍是 `video_queue_frames: 5`。它不再在截止时间前 `pread` SSD，只向 ahead cache 要已经在内存里的帧。10 Hz 源重复发 3 次时继续共用同一个 `shared_ptr`。
- `orchestrator_main`：`video_prefetch_ms > 0` 时 `stage_to_shm = false`，不调用 `apply_staged_payload_paths`。`video_prefetch_ms == 0` 时保持现在的整段 stage（`play_duration + 1s`）。

- [ ] **Step 1: 写失败测试**

用和 `test_video_prefault.cpp` 一样的小帧（16 字节）造 1 秒、10 Hz / 30 Hz 时间戳。`prefetch_ns = 200ms` 时：

- 起点只读入落在前 200 ms 的源帧。
- 200 ms 之后的源帧偏移没有被读过。
- `release_before` 掉已经播过的索引后，可以再读入下一截，同时驻留帧数不超过 200 ms 所覆盖的帧数。

- [ ] **Step 2: 跑测试，确认失败**

- [ ] **Step 3: 实现上面的类，并接到 producer**

`producer_main` 在 `config_.video_prefetch_ms > 0` 时走 `ahead->acquire`，不要再对 `payload_path` 做播放期 `pread`。

- [ ] **Step 4: 预读测试和原有 `test_video_prefault` 都通过**

原有整段 stage 测试必须仍然通过。

### Task 3: 40 秒配置

**Files:**
- Create: `joint_sensor_player/config/orchestrator_xdma_joint_40s.yaml`
- Modify: `joint_sensor_player/README.md`（配置列表加一行）

从 `orchestrator_xdma_joint_10s_shm.yaml` 复制，只改这些：

- `play_duration_ms: 0`（播共同窗口的完整段，约 39.999 秒；时长线程在 0 时不会提前停）
- `video_prefetch_ms: 5000`
- 注释写明：不把视频窗口 stage 到 `/dev/shm`；点云仍用 `/dev/shm/joint_radar_data`
- `video_max_lateness_us: 5000`、`lidar_max_lateness_us: 100000`、`start_delay_ms: 5000`、`wait_for_enter: true` 保持不变
- `lanes` 仍是 tx1 / tx2 / tx4，不要加周视

- [ ] **Step 1: 配置加载测试读这个 YAML，断言 `play_duration_ms == 0` 且 `video_prefetch_ms == 5000`**

- [ ] **Step 2: 跑该测试**

### Task 4: 本机测试，不在这一步做实机 40 秒

- [ ] **Step 1: 跑 player 与 orchestrator 的现有测试，加上 Task 2、Task 3 的新测试**

Release 构建后跑对应 ctest。10 秒配置的测试不能坏。

- [ ] **Step 2: 不要自行启动注入卡或域控。** 实机 40 秒等用户明确说再跑。跑的时候用：

`./build-release/joint_sensor_orchestrator --config config/orchestrator_xdma_joint_40s.yaml`

通过标准：`ok=true`，视频每路最大迟到不超过 5000 µs，雷达不超过 100000 µs，报告里 `staged_bytes=0`，预读驻留不超过约 5 秒的源帧。
