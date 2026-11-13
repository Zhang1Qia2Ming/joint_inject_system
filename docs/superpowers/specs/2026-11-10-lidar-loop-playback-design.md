# 八路雷达轮询播放设计

## 目标

在现有严格1倍速八路雷达播放器上增加连续轮询能力，用于长时间压测ThorA
的`lidar_service`。单次播放保持默认行为，压测模式可无限循环，且不能在
轮次边界制造突发、额外停顿或UDP序号倒退。

## 配置契约

```yaml
playback:
  loop_count: 1
  sequence_mode: continuous
```

- `loop_count: 1`：默认单次播放，兼容当前行为。
- `loop_count: 0`：无限循环，直到收到`SIGINT`或`SIGTERM`。
- `loop_count: N`：总共播放N轮，N必须为非负整数。
- `sequence_mode: continuous`：跨轮连续化雷达UDP包序号。
- `sequence_mode: preserve`：每轮原样发送UDP，不改序号。

## 连续时间轴

MCAP只扫描和解码一次。每轮复用同一个目录和共同播放窗口，不重新创建
socket或重新读取文件。

循环周期为：

```text
cycle_period = common_source_end - common_source_start + nominal_frame_period
```

八路雷达均为约10 Hz。`nominal_frame_period`使用所有启用topic相邻帧间隔
的中位数计算，并要求处于80–120 ms；否则拒绝启动。第N轮的包期限为：

```text
play_start
+ N × cycle_period
+ (frame_source_time - common_source_start)
+ packet_index × packet_interval
+ channel_time_offset
```

这样短雷达上一轮末包与下一轮首包保持约一个正常包间空隙，主雷达也保持
原有帧内、帧间节奏。

## UDP序号连续化

连续模式只修改内存中的UDP发送副本，不修改MCAP文件、点云内容、包内时间
字段或调度源时间。

协议适配器：

- 主雷达：识别`EE FF 04 03`，按AT128 UDP 4.3结构计算序号偏移。
- 补盲雷达：识别554字节且头部为`EE FF 07 03`，序号偏移为530。

启动前扫描每个启用topic的所有包：

1. 每个包都必须被对应适配器识别。
2. 同一通道序号必须按uint32模运算单调前进，不允许倒退。
3. 记录第一包、最后一包和一轮序号跨度。

第一轮保持原序号。后续每发送一个包，将当前内存包序号增加一轮跨度；
uint32自然回绕。此方式保留素材内部已有的序号间隙，只消除轮次边界倒退。
`preserve`模式完全跳过协议识别和改写。

## 停止与错误

- `SIGINT`、`SIGTERM`设置无锁停止标志；调度器在每包发送前检查。
- 用户停止属于正常结束，报告中记录停止原因，不标记发送失败。
- 任一路非阻塞UDP发送失败、迟到超过20 ms或序号识别/连续性校验失败时，
  立即停止全部八路。
- 无限循环使用uint64轮次计数；轮次或期限计算溢出时停止并报错。

## 报告

不增加逐轮压测报告。沿用现有累计报告：

- 累计帧数、包数；
- 最大迟到；
- 报告队列丢弃数；
- 最终停止原因。

`summary.json`增加`loops_completed`和`stop_reason`，不记录每轮明细。

## 测试

- 默认配置仍只播放一轮。
- 两轮播放的第二轮首帧期限连续且无额外间隔。
- `loop_count: 0`可由取消标志停止。
- preserve模式不修改任何payload字节。
- 主雷达4.3、补盲7.3序号跨轮连续并支持uint32回绕。
- 不识别协议、素材内序号倒退、发送失败和迟到超限均停止全部通道。
- 真实40秒MCAP可通过dry-run序号预检。
