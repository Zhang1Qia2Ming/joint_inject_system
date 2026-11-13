# joint_sensor_player

一期实现八路雷达MCAP严格1倍速及连续轮询回放，并新增独立
`joint_sensor_orchestrator`完成八雷达与TX1/TX2/TX4三路视频的一次性联合
调度。八路共用`enp3s0f0`，使用
`192.168.1.31-38:10000`模拟真实雷达，向ThorA
`192.168.1.4:2335-2338/2368-2371`发送原始UDP。

播放器不会修改MCAP或包内时间戳；单次播放保持UDP原样，循环
`continuous`模式只修改内存发送副本中的4字节包序号。相机联合回放接口
将在视频注入卡接入后扩展。

`hil_lidar_emulator`是独立的特殊真雷达模拟器：它在发送副本中同时连续化
UDP4.3/7.3包内UTC、微秒时间、序号和主雷达尾部CRC，用于无限轮询压测时
消除包内时间回退；同时在八个模拟雷达IP的TCP 9347端口提供PTC correction
标定服务，保证域控可像连接真雷达一样完成八路点云解码。原播放器行为不受
影响。

## 目录

```text
~/zhang/joint_sensor_player/       代码、配置、报告
/data/joint_sensor_player/data/    MCAP文件（只扫描第一层）
```

把同一批次的MCAP直接放进`/data/joint_sensor_player/data/`。程序根据
topic识别八路雷达，不依赖A/B文件名。同一topic存在多个分段时按内部时间戳
合并；完全重复帧去重，时间戳相同但内容不同则拒绝启动。

## 构建

```bash
cd ~/zhang/joint_sensor_player
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

依赖：`g++`、`cmake`、`libyaml-cpp-dev`、`libzstd-dev`和
`nlohmann-json3-dev`。

## 操作

先确认真实雷达已经断网，避免`.31-.38`地址冲突：

```bash
cd ~/zhang/joint_sensor_player
./scripts/prepare_network.sh
./scripts/prepare_network.sh --check
```

仅扫描、校验，不发送：

```bash
./build/joint_sensor_player --config config/example.yaml --dry-run
```

正式一次性1倍速回放（默认`loop_count: 1`）：

```bash
./build/joint_sensor_player --config config/example.yaml
```

无限循环压测`lidar_service`：

```bash
./build/joint_sensor_player --config config/stress.yaml
```

无限循环并模拟真雷达连续时间：

```bash
./build/hil_lidar_emulator --config config/hil_emulator.yaml --dry-run
./build/hil_lidar_emulator --config config/hil_emulator.yaml
```

`hil_emulator.yaml`中的每路`correction_file`必须对应当前MCAP内嵌标定；
PTC服务默认配置如下：

```yaml
ptc:
  enabled: true
  port: 9347
```

模拟器默认要求本机NTP已同步。仅在不关心绝对延迟的离线环境中，才可显式
设置：

```yaml
clock:
  require_synchronized: false
```

该降级只消除时间回退，不能保证上位机与域控绝对时钟一致。

## 联合播放器

默认配置`config/orchestrator_mock.yaml`固定`loop_count: 1`，视频后端为
Mock，三路队列各五帧。配置使用当前批次真实雷达/视频路径、三个H.265 topic
及八路correction文件。只有把`mode.video_backend`明确改为`xdma`时程序才会
发现并打开`/dev/xdma*_h2c_0..2`；默认运行不会访问视频硬件。

### XDMA 阶段进度（截至 2026-11-11）

| 项 | 状态 |
|---|---|
| Mock 联合 10s（雷达 shm + Mock 视频） | 已稳定 PASS |
| XDMA 仅视频冒烟 ~10s | 已通过（含 prefault） |
| XDMA + 八路雷达联合 10s | 已稳定（无 mlock 后再压 5/5 PASS；雷达 shm + NVMe UYVY + `/dev/shm` stage） |
| 雷达迟到门限（XDMA 联合 10s shm 配置） | 100ms（视频仍 5ms；主机 swap/视频 pread 争用下见过 ~50ms 包级尖峰） |
| ThorA / `thor_nvsipl` | 已跑通：`img_transfer_v2` 三路有图；预览故意 `CS_HIL_PREVIEW_FPS=10`，**不能**用预览 FPS 否定 30 Hz 注入（证据目录见下） |
| ThorB 雷达内部同步 | 同次窗口 88 帧融合逐条统计仍 **sync8=0%**（主雷交替 2+2、补盲齐）；**非**因不足 1 分钟；不定 `physical_sync` |
| 物理同步 / FSYNC / 双卡七路 | 仍 `NOT_RUN`（本轮不定 `physical_sync=PASS`） |
| 完整 ~40s / 全量窗口 | 未做（shm 约 16G，10s 窗口已占 ~9.1G，40s 需另评估） |

关键修复：流式雷达按 catalog 前瞻入队；XDMA pread 限速+串行；`play_duration`
从 `play_start` 起算；播放前把窗口帧 stage 到 `/dev/shm` 稀疏文件并对原盘
`DONTNEED`。窗口 **不做 mlock**（RLIMIT 只能锁住约 3.8G/9.1G，mlock 会把
雷达迟到顶到 25ms 门限附近）。

### 真实注入卡怎么跑

不要改默认 `orchestrator_mock.yaml`。常用配置：

- `config/orchestrator_xdma_smoke.yaml`：仅视频冒烟 ~10s
- `config/orchestrator_xdma_joint_10s_shm.yaml`：**推荐** 八路雷达(`/dev/shm`) + 三路 XDMA ~10s
- `config/orchestrator_xdma_joint_40s.yaml`：同一张卡的三路完整约 40 秒。视频留在 SSD，内存只预读 5 秒；点云仍在 `/dev/shm`
- `config/orchestrator_xdma_joint_10s.yaml`：雷达走磁盘 data_dir 的 10s 联合
- `config/orchestrator_xdma.yaml`：全窗口联合（尚未按 10s 矩阵同等压测）

**每次开机 / 换机后先确认：**

1. 网口与 31–38 源 IP：`./scripts/prepare_network.sh --check`（缺则按脚本恢复）
2. 注入卡空闲：`./scripts/check_xdma_conflicts.sh`（有 `pcie_uyvy` 先停掉）
3. UYVY 在 NVMe：`config` 里 `cache_dir: /var/tmp/joint_sensor_player/cache`  
   若目录不存在，从 `/data/joint_sensor_player/cache/<key>/` 拷到该路径（约 31G）
4. 雷达 10s 矩阵：`/dev/shm/joint_radar_data` 已有 mcap（约 1.1G）
5. `/dev/shm` 剩余空间 ≳ 10G（窗口暂存约 9.1G + 雷达 1.1G）
6. 用 **Release** 二进制：`./build-release/joint_sensor_orchestrator`

```bash
cmake --build build-release -j"$(nproc)" --target joint_sensor_orchestrator

./scripts/prepare_network.sh --check
./scripts/check_xdma_conflicts.sh

# 1) 仅视频冒烟
./build-release/joint_sensor_orchestrator \
  --config config/orchestrator_xdma_smoke.yaml

# 2) 推荐：10s 联合（雷达 shm + XDMA）
./build-release/joint_sensor_orchestrator \
  --config config/orchestrator_xdma_joint_10s_shm.yaml
```

终端会持续打印`状态:`。准备视频和等待开播时带`尚未发送`。`wait_for_enter: true` 时，倒计时结束后停在`等待回车`，倒计时期间按过的回车不算。看到这行后再按一次回车，才按 `start_delay_ms` 预填队列，终端显示`正在预填发送队列`。真正开播时打印`发送开始:`和本地时间，之后每秒打印`状态: 发送中`。结束时打印`发送结束:`、`发送时长:`，再打印`STOPPED(duration_limit) report=".../reports/orchestrator-..."`。  
看报告：`summary.json` 中 `ok: true`，`validation.software_logic/video_payload_io/xdma_device` 为 `PASS`；  
`run.log` 含 `staged_bytes=9140...`。失败时先看 `stop_reason` / `first_failure`。

`check_xdma_conflicts.sh`只检查并提示占用`/dev/xdma0_h2c_*`或`pcie_uyvy`
的进程，不会自动强杀。无 Thor 证据时 `thor_nvsipl` / `physical_sync` 保持
`NOT_RUN`。物理同步、FSYNC、双卡七路仍为`NOT_RUN`。

### Thor 核对（SoC A 视频 + SoC B 雷达，10s 联合）

约定：相机进 A、雷达进 B；本轮验证**接收与雷达内部同步**，不定雷视
`physical_sync=PASS`；不做 31–38 UDP 抓包；不碰生产 `camera_service`
（`host_uyvy` 未打通）。UTC = 主机写入雷达包内的时间轴，不等于域控 `pub`，
也不等于网卡到达墙钟。

**顺序：**

1. **主机预备**（`~/zhang/joint_sensor_player`）  
   `./scripts/prepare_network.sh --check`  
   `./scripts/check_xdma_conflicts.sh`

2. **SoC A**（`~/zhang/enter_thorA_through_TBOX.sh`）  
   ```bash
   ps -ef | grep -E '[c]amera_service|[e]xecution_start' || true
   systemctl stop execution_start   # 若仍在
   cd /root/img_transfer_v2 && ./main.sh start && ./main.sh status
   ```  
   本机可转发网页：  
   `ssh -J root@172.31.17.7 -L 8089:127.0.0.1:8089 root@172.31.8.9`  
   → `http://127.0.0.1:8089/` 看 Sensor8/10/11 FPS 与快照。

3. **主机联合 10s**（A 预览已起来后再跑）  
   ```bash
   ./build-release/joint_sensor_orchestrator \
     --config config/orchestrator_xdma_joint_10s_shm.yaml
   ```  
   记录 `reports/orchestrator-...`（`ok`、`utc_anchor_us`、视频/雷达迟到）。

4. **SoC A 取证**  
   `./main.sh status`；确认 8/10/11 有稳定 ~30 FPS；保留  
   `/root/img_transfer_v2/output/sensor{8,10,11}.jpg` 与 `/run/img_transfer_v2/nvsipl.log` 片段。  
   结束后：`./main.sh stop`（会 recover 注入环境并视情况拉起 `execution_start`）。

5. **SoC B**（`~/zhang/enter_thorB_through_TBOX.sh`，经 A 双跳）  
   日志通常在 `/opt/update/log/lidar_service/`（以板上为准）。  
   ```bash
   python3 /root/analyze_sync_rate_and_delay.py \
     /opt/update/log/lidar_service/lidar_service.log \
     --exclude-seconds 0 --last 1
   ```  
   （窗口按实际播放墙钟裁；10s 很短，可改小 exclude / 用 `--last 1`。）  
   看八路同步率、各雷达有效率、包内时间跨度；与 11-10 结论对照。

6. **报告层级**  
   - 主机 `ok` + XDMA：`software_logic` / `video_payload_io` / `xdma_device`  
   - A 收流证据齐：才可议 `thor_nvsipl=PASS`  
   - B 脚本结果：雷达内部同步笔记（不自动等于雷视同步）  
   - `physical_sync` / FSYNC：本轮保持 `NOT_RUN`

安全生成视频缓存（Python通过`fork/execv`参数数组启动，不经过shell；成功后
C++会重新校验manifest、路径、长度、时间戳和SHA-256）：

```bash
./build/joint_sensor_orchestrator \
  --config config/orchestrator_mock.yaml --prepare-cache
```

仅扫描雷达、校验缓存和共同时间窗口，不创建UDP/XDMA发送端：

```bash
./build/joint_sensor_orchestrator \
  --config config/orchestrator_mock.yaml --dry-run
```

执行一次联合播放：

```bash
./build/joint_sensor_orchestrator --config config/orchestrator_mock.yaml
```

Mock后端会先完成缓存manifest、路径、长度、时间戳及完整UYVY文件SHA-256
校验，但播放期间不会逐帧读取UYVY；它使用尺寸正确的合成帧缓冲，通过同一
有界队列验证槽映射、调度、计数、迟到和故障联停。只有XDMA后端会逐帧读取
真实UYVY缓存并写入硬件。报告中的`lanes[].source_frames`表示纳入调度的
源内容帧，不表示已从磁盘读取。

协调器仅在1个雷达线程和3组视频生产/消费线程（共7个工作线程）全部READY后
释放共同StartGate并发布统一单调时钟起点和UTC锚点。任一准备或运行故障会
全局取消、join并停止PTC/视频sink；`Ctrl-C`/`SIGTERM`按正常STOPPED收敛。

联合报告独立使用schema v2，输出`summary.json`、`video_frames.csv`、
兼容雷达列格式的`frame_stats.csv`、`anomalies.jsonl`和`run.log`。Mock运行
成功才记录`software_logic: PASS`，失败记录`FAIL`；`--dry-run`明确记录
`software_logic: NOT_RUN`，真实运行被`SIGINT`/`SIGTERM`正常取消时记录
`software_logic: INCOMPLETE`，不会误报PASS。summary保持`schema_version: 2`，
并用`payload_mode: synthetic`明确标识Mock合成载荷；此时
`validation.video_payload_io`为`NOT_RUN`，不能解释为真实UYVY I/O通过。
只有XDMA完整成功运行才会把该层记为`PASS`。`xdma_device`、`thor_nvsipl`、
`physical_sync`和`hardware_fsync`仍保持`NOT_RUN`，除非有对应独立验证。
现有字段保持兼容。summary使用临时文件、`fsync`和原子rename发布；源MCAP
仅mtime变化时仍以SHA-256安全复用缓存，并在summary与run log记录诊断元数据。

按`Ctrl-C`停止。`config/stress.yaml`使用：

```yaml
playback:
  loop_count: 0
  sequence_mode: continuous
```

`loop_count: 0`表示无限循环，正数表示总播放轮数。轮次之间不增加停顿，
下一轮仍按约10 Hz连续衔接。`continuous`只连续化内存发送副本中的UDP包
序号，不修改MCAP、点云内容或包内时间。需要对照原始重播行为时可改为
`sequence_mode: preserve`。

测试结束删除脚本添加的地址：

```bash
./scripts/prepare_network.sh --restore
```

## 启动门禁

- 网口必须有载波、至少1000 Mbps且为全双工。
- 八个已启用topic必须全部存在，默认不允许部分启动。
- 八个源IP必须已经配置在目标网口。
- 八路数据必须存在共同时间区间。
- 同一topic同一时间戳的冲突数据会中止启动。
- 循环连续模式要求主雷达UDP 4.3和补盲UDP 7.3序号格式全部通过预检。
- 真雷达模拟器还会校验每包时间严格递增、循环周期足够、系统时钟同步及
  UDP4.3尾部CRC，并要求八路correction文件存在；任一路失败都不会发送。
- 任一路迟到超过20 ms、发送失败或序号异常都会停止全部八路。

## 报告

每次运行在`reports/<启动时间>/`生成：

- `summary.json`：稳定的机器可读汇总格式，当前schema版本为1。
- `frame_stats.csv`：按雷达帧记录计划及实际末包时间。
- `anomalies.jsonl`：分段间隙等异常。
- `config_snapshot.yaml`：本次运行配置快照。

报告线程使用有限队列，调度线程只做非阻塞提交。队列满时丢弃普通统计并在
`summary.json`记录数量，不阻塞雷达发送。
