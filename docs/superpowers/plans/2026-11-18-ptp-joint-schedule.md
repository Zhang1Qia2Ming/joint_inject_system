# PTP 联合时间表 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改系统时间、不把 D10 接回编排器的前提下，让八路雷达和三路视频按域控 gPTP 排开发送时刻。

**Architecture:** `ptp4l` 只做 `enp3s0f0` 的从钟，锁定 `/dev/ptp1`。编排器采样一次「单调钟与这只硬件钟的差」。前雷达第一包的发送时刻向上取整到硬件钟的 100 ms 网格，所有雷达包和视频内容时刻加同一段位移。视频写卡期限是推算 D10 上升沿再提前 20 ms。D10 相位来自一次 ILA 测量，误差按毫秒计。

**Tech Stack:** C++17，现有 `joint_sensor_player_tests`，linuxptp `ptp4l`，`/dev/ptp1`。

## Global Constraints

- 视频迟到门限保持 5000 µs，雷达迟到门限保持 100000 µs。不放宽。
- `physical_sync` 和 `hardware_fsync` 保持 `NOT_RUN`。毫秒级相位不能标成通过。
- 不运行 `phc2sys`，不用 `/dev/ptp1` 改 Linux 系统时间。
- 不重新启用 `video_engine.cpp` 里等 `/dev/xdma0_events_0` 的起跑逻辑。D10 不接回编排器。
- 八路雷达继续共用 `enp3s0f0`，源地址 `192.168.1.31`–`.38`。
- 不改 `lidar_service`，不改生产 `camera_service`。
- 丢拍规则不变：空闲且帧未写完就丢掉这一拍，不保留沿，不重复旧帧。
- 不把光栅和 GMSL 的 33.3 ms 从视频时间里扣掉。
- 相位文件必须写明 `uncertainty=millisecond`。缺这个字段就拒绝启用。
- 默认关闭。配置没打开时，现有期限一个数都不改。

---

### Task 1: 纯时间表

**Files:**
- Create: `include/joint_sensor_orchestrator/ptp_schedule.hpp`
- Create: `src/ptp_schedule.cpp`
- Test: `tests/test_ptp_schedule.cpp`
- Modify: `CMakeLists.txt`（把两个新 cpp 加进对应目标）

**Interfaces:**
- Consumes: 无
- Produces:
  - `struct PtpSample { std::uint64_t monotonic_ns; std::uint64_t phc_ns; };`
  - `std::uint64_t mono_to_phc(std::uint64_t mono_ns, const PtpSample& sample);`
  - `std::uint64_t phc_to_mono(std::uint64_t phc_ns, const PtpSample& sample);`
  - `std::uint64_t shared_shift_ns(std::uint64_t first_front_deadline_mono, const PtpSample& sample);`
  - `std::uint64_t nearest_d10_edge_phc(std::uint64_t content_phc_ns, std::uint64_t phase_mod_ns, std::uint64_t period_ns);`
  - `std::uint64_t video_write_deadline_mono(std::uint64_t content_deadline_mono, std::uint64_t shared_shift_ns, const PtpSample& sample, std::uint64_t phase_mod_ns, std::uint64_t period_ns, std::uint64_t lead_ns);`
  - 常量：`kLidarGridNs = 100000000`，`kD10PeriodNs = 33334333`，`kVideoLeadNs = 20000000`

- [ ] **Step 1: 写失败测试**

`tests/test_ptp_schedule.cpp`：

```cpp
#include "joint_sensor_orchestrator/ptp_schedule.hpp"
#include "test_framework.hpp"

using joint_sensor_orchestrator::PtpSample;
using joint_sensor_orchestrator::kD10PeriodNs;
using joint_sensor_orchestrator::kLidarGridNs;
using joint_sensor_orchestrator::kVideoLeadNs;
using joint_sensor_orchestrator::mono_to_phc;
using joint_sensor_orchestrator::nearest_d10_edge_phc;
using joint_sensor_orchestrator::shared_shift_ns;
using joint_sensor_orchestrator::video_write_deadline_mono;

TEST_CASE(shift_moves_front_lidar_onto_next_100ms) {
  const PtpSample sample{1'000'000'000ULL, 5'000'000'000ULL};
  const auto deadline = sample.monotonic_ns + 50'000'000ULL;
  const auto shift = shared_shift_ns(deadline, sample);
  CHECK(shift == 50'000'000ULL);
  CHECK(mono_to_phc(deadline + shift, sample) % kLidarGridNs == 0);
}

TEST_CASE(already_aligned_shift_is_zero) {
  const PtpSample sample{1'000'000'000ULL, 5'000'000'000ULL};
  CHECK(shared_shift_ns(sample.monotonic_ns, sample) == 0);
}

TEST_CASE(nearest_edge_picks_closer_side) {
  const auto period = kD10PeriodNs;
  const auto phase = 1'000'000ULL;
  CHECK(nearest_d10_edge_phc(phase + 100ULL, phase, period) == phase);
  CHECK(nearest_d10_edge_phc(phase + period - 100ULL, phase, period) ==
        phase + period);
}

TEST_CASE(video_write_is_20ms_before_predicted_edge) {
  const PtpSample sample{0, 0};
  const auto content = kD10PeriodNs;
  const auto write = video_write_deadline_mono(
      content, 0, sample, 0, kD10PeriodNs, kVideoLeadNs);
  CHECK(write == content - kVideoLeadNs);
}
```

- [ ] **Step 2: 跑测试，确认失败**

Run: `cmake --build build-release -j"$(nproc)" --target joint_sensor_player_tests && ./build-release/joint_sensor_player_tests ptp_schedule`

Expected: 编译失败，`ptp_schedule.hpp` 不存在。

- [ ] **Step 3: 实现**

`shared_shift_ns` 把前雷达第一包的单调期限换成 PHC，再向上取整到 100 ms。已经落在整点上时位移为 0。位移范围是 `[0, 100ms)`。

`nearest_d10_edge_phc` 在 `phase_mod + n * period` 里选离 `content_phc_ns` 最近的一拍。距离相等时取较早的那一拍。`phase_mod_ns >= period_ns` 抛 `std::invalid_argument`。

`video_write_deadline_mono` 先把内容期限加上 `shared_shift_ns`，换到 PHC，取最近的 D10 沿，再换回单调钟并减去 `lead_ns`。结果早于 0 则抛 `std::overflow_error`。

- [ ] **Step 4: 再跑，确认通过**

Run: `cmake --build build-release -j"$(nproc)" --target joint_sensor_player_tests && ./build-release/joint_sensor_player_tests ptp_schedule`

Expected: 四个 `TEST_CASE` 都通过。

- [ ] **Step 5: Commit**

```bash
git add include/joint_sensor_orchestrator/ptp_schedule.hpp src/ptp_schedule.cpp tests/test_ptp_schedule.cpp CMakeLists.txt
git commit -m "$(cat <<'EOF'
加 PTP 联合时间表的纯计算。

前雷达对齐 100 毫秒网格，视频写卡期限在推算的 D10 沿之前。

EOF
)"
```

### Task 2: 相位文件

**Files:**
- Modify: `include/joint_sensor_orchestrator/ptp_schedule.hpp`
- Modify: `src/ptp_schedule.cpp`
- Modify: `tests/test_ptp_schedule.cpp`

**Interfaces:**
- Consumes: Task 1 的常量
- Produces: `struct D10Phase { std::uint64_t phase_mod_ns; std::uint64_t period_ns; std::uint64_t lead_ns; };` 和 `D10Phase load_d10_phase(const std::filesystem::path& path);`

- [ ] **Step 1: 写失败测试**

文件内容必须同时有这四行，顺序不限：

```text
phase_mod_ns=1000000
period_ns=33334333
lead_ns=20000000
uncertainty=millisecond
```

`uncertainty` 不是 `millisecond` 时 `load_d10_phase` 抛 `std::runtime_error`，错误文本包含 `毫秒级`。缺任意一行同样抛。

- [ ] **Step 2: 跑测试，确认失败**

Run: `cmake --build build-release -j"$(nproc)" --target joint_sensor_player_tests && ./build-release/joint_sensor_player_tests d10_phase`

Expected: `load_d10_phase` 未声明，编译失败。

- [ ] **Step 3: 实现**

逐行读 `key=value`。`phase_mod_ns` 必须小于 `period_ns`。`lead_ns` 读到的值必须等于 `kVideoLeadNs`。不接受微秒级声明。

- [ ] **Step 4: 再跑，确认通过**

Expected: 合法文件读出 `phase_mod_ns == 1000000`。把 `uncertainty=microsecond` 的临时文件交给 `load_d10_phase` 时抛异常。

- [ ] **Step 5: Commit**

```bash
git add include/joint_sensor_orchestrator/ptp_schedule.hpp src/ptp_schedule.cpp tests/test_ptp_schedule.cpp
git commit -m "$(cat <<'EOF'
相位文件只接受毫秒级标记。

缺标记或写成微秒就拒绝启用。

EOF
)"
```

### Task 3: 接到编排器，默认关闭

**Files:**
- Modify: `include/joint_sensor_orchestrator/orchestrator_config.hpp`
- Modify: `src/orchestrator_config.cpp`
- Modify: `src/orchestrator_main.cpp`
- Modify: `tests/test_orchestrator_config.cpp`
- 不改 `src/video_engine.cpp` 里现有的 `shared_fsync_reference`

**Interfaces:**
- Consumes: `load_d10_phase`、`shared_shift_ns`、`video_write_deadline_mono`
- Produces: 配置字段 `bool ptp_schedule{false};` 和 `std::filesystem::path d10_phase_file;`

打开时还要读一次硬件钟。读钟函数放在 `src/ptp_schedule.cpp`：

```cpp
PtpSample read_ptp_sample(const std::filesystem::path& device);
```

打开 `device`（默认 `/dev/ptp1`），用 `FD_TO_CLOCKID` 调用 `clock_gettime`，同时读 `CLOCK_MONOTONIC`。打不开或读失败抛 `std::runtime_error`。单元测试不打开真设备。

- [ ] **Step 1: 写配置测试**

`ptp_schedule: false` 时字段为 false。`ptp_schedule: true` 且没有 `d10_phase_file` 时 `load_orchestrator_config` 抛异常。YAML 里的 `video_max_lateness_us: 5000` 和 `lidar_max_lateness_us: 100000` 保持原值，测试里断言这两个数不被新字段改掉。

- [ ] **Step 2: 跑测试，确认失败**

Expected: 未知字段或新字段不存在，测试失败。

- [ ] **Step 3: 接入发送期限**

`ptp_schedule == false` 时，`build_video_slots` 和 `packet_deadline_ns` 的调用参数与现在一致。

`ptp_schedule == true` 时：

1. `load_d10_phase`。
2. `read_ptp_sample("/dev/ptp1")`。失败就停止，不退回旧时间表。
3. 用前雷达 topic `/sensor/lidar_front/orig/udp_packets` 的第一包、`packet_index == 0` 的现有期限计算 `shared_shift_ns`。
4. 八路雷达每个包的期限都加上这段位移。包与包的间隔不加第二次。
5. 每路视频每个槽：内容期限先加同一段位移，再换成 `video_write_deadline_mono`。`slot_time_ns` 和 `source_time_ns` 不改，报告里仍是源时间。
6. 不调用 `shared_fsync_reference`。不打开 `/dev/xdma0_events_0`。

`run.log` 增加一行：`ptp_schedule shift_ns=<n> uncertainty=millisecond`。`summary.json` 的 `hardware_fsync` 和 `physical_sync` 仍写 `NOT_RUN`。

- [ ] **Step 4: 跑编排器测试**

Run: `cmake --build build-release -j"$(nproc)" --target joint_sensor_player_tests && ./build-release/joint_sensor_player_tests`

Expected: 现有测试通过。默认配置的期限与改动前相同。

- [ ] **Step 5: Commit**

```bash
git add include/joint_sensor_orchestrator/orchestrator_config.hpp src/orchestrator_config.cpp src/orchestrator_main.cpp src/ptp_schedule.cpp include/joint_sensor_orchestrator/ptp_schedule.hpp tests/test_orchestrator_config.cpp
git commit -m "$(cat <<'EOF'
联合播放可按 PTP 时间表发送，默认关闭。

打开时才读取相位文件和硬件钟，报告仍不标同步通过。

EOF
)"
```

### Task 4: 从钟脚本和测量步骤

**Files:**
- Create: `config/ptp4l-enp3s0f0-slave.cfg`
- Create: `scripts/ptp_slave_enp3s0f0.sh`
- Modify: `docs/2026-11-13-joint-video-lidar.md`（补操作命令，风险句已经在）

**Interfaces:**
- Consumes: 无代码接口
- Produces: 操作者手动启动的从钟。播放器不调用这个脚本。

- [ ] **Step 1: 写配置**

`config/ptp4l-enp3s0f0-slave.cfg`：

```text
[global]
domainNumber            0
slaveOnly               1
BMCA                    noop
inhibit_announce        1
network_transport       L2
delay_mechanism         P2P
transportSpecific       0x1
ptp_dst_mac             01:80:C2:00:00:0E
time_stamping           hardware
step_threshold          1
[enp3s0f0]
```

不写 `acceptedGrandmasterIdentity`。不写 VCAR 的 `02805e.fffe.1f0008`。

- [ ] **Step 2: 写脚本**

`scripts/ptp_slave_enp3s0f0.sh` 只执行：

```bash
exec sudo ptp4l -i enp3s0f0 -H -s -f config/ptp4l-enp3s0f0-slave.cfg -m
```

脚本注释写明：不要再启动 `phc2sys`。日志里出现 `s2` 且 offset 稳定在微秒内，才算从钟锁上。锁上之前不要打开 `ptp_schedule`。

- [ ] **Step 3: 测量步骤写进联合播放文档**

测量由人来做，程序不自动抓 ILA：

1. 从钟锁定后，用现有 `capture_d10_ila.sh` 抓 D10 上升沿。
2. 触发回到主机时读 `/dev/ptp1`。
3. 相位取 `phc_ns % 33334333`。
4. 写成 Task 2 的四行文件，`uncertainty` 固定 `millisecond`。
5. 文档保留这句话：这次相位是毫秒级，不是微秒级。JTAG 往返把误差留在毫秒。20 ms 提前量只用来盖住这个误差。

- [ ] **Step 4: 不在本任务里开播**

不把 `orchestrator_xdma_joint_40s_shadow4.yaml` 的 `ptp_schedule` 改成 true。等相位文件和从钟日志都在，再单独改配置。

- [ ] **Step 5: Commit**

```bash
git add config/ptp4l-enp3s0f0-slave.cfg scripts/ptp_slave_enp3s0f0.sh docs/2026-11-13-joint-video-lidar.md
git commit -m "$(cat <<'EOF'
加上 enp3s0f0 的 gPTP 从钟脚本和毫秒级测量步骤。

播放器不改系统时间，也不默认启用新时间表。

EOF
)"
```
