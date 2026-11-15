#include "joint_sensor_orchestrator/enter_wait.hpp"
// 联合播放的进程入口。
// 先核对视频缓存和雷达目录的共同窗口，再按配置把视频放进内存或留下滑动窗口。
// 点云单独 mlock。环视模拟如果打开，会在 orchestrator.run 之前把排期帧读完。
// 真正的对齐和发送在 Orchestrator::run 里。失败也会把报告留在 reports 目录。

#include "joint_sensor_orchestrator/cache_preparer.hpp"
#include "joint_sensor_orchestrator/dataset_validation.hpp"
#include "joint_sensor_orchestrator/orchestrator.hpp"
#include "joint_sensor_orchestrator/playback_status.hpp"
#include "joint_sensor_orchestrator/orchestrator_config.hpp"
#include "joint_sensor_orchestrator/process.hpp"
#include "joint_sensor_orchestrator/radar_engine.hpp"
#include "joint_sensor_orchestrator/report_failure_guard.hpp"
#include "joint_sensor_orchestrator/report_v2.hpp"
#include "joint_sensor_orchestrator/smoke_radar.hpp"
#include "joint_sensor_orchestrator/file_pin.hpp"
#include "joint_sensor_orchestrator/thread_affinity.hpp"
#include "joint_sensor_orchestrator/video_cache.hpp"
#include "joint_sensor_orchestrator/video_engine.hpp"
#include "joint_sensor_orchestrator/video_prefault.hpp"
#include "joint_sensor_orchestrator/shadow_video.hpp"
#include "joint_sensor_orchestrator/video_sink.hpp"
#include "joint_sensor_player/mcap.hpp"
#include "joint_sensor_player/timeline.hpp"
#include "joint_sensor_player/udp_sender.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <sys/resource.h>
#include <sys/select.h>
#include <unistd.h>

namespace {

using namespace joint_sensor_orchestrator;

volatile std::sig_atomic_t g_signal_stop = 0;
void handle_stop_signal(int) { g_signal_stop = 1; }

struct Arguments {
  std::filesystem::path config;
  bool prepare_cache{};
  bool dry_run{};
};

void usage(std::ostream& output) {
  output
      << "用法: joint_sensor_orchestrator --config FILE "
         "[--prepare-cache] [--dry-run]\n"
         "  --prepare-cache  用安全参数数组调用Python生成并校验缓存后退出\n"
         "  --dry-run        仅扫描MCAP、缓存和共同窗口，不发送任何数据\n";
}

Arguments parse_arguments(int argc, char** argv) {
  Arguments result;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      usage(std::cout);
      std::exit(0);
    }
    if (argument == "--prepare-cache") {
      result.prepare_cache = true;
    } else if (argument == "--dry-run") {
      result.dry_run = true;
    } else if (argument == "--config" && index + 1 < argc) {
      result.config = argv[++index];
    } else {
      throw std::runtime_error("未知或不完整参数: " + argument);
    }
  }
  if (result.config.empty()) throw std::runtime_error("缺少--config FILE");
  if (result.prepare_cache && result.dry_run) {
    throw std::runtime_error("--prepare-cache与--dry-run不能同时使用");
  }
  return result;
}

std::filesystem::path create_run_directory(
    const std::filesystem::path& root) {
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
  localtime_r(&time, &local);
  std::ostringstream base;
  base << "orchestrator-" << std::put_time(&local, "%Y%m%d-%H%M%S");
  std::filesystem::create_directories(root);
  for (unsigned suffix = 0; suffix < 1000; ++suffix) {
    const auto path =
        root / (base.str() + (suffix ? "-" + std::to_string(suffix) : ""));
    if (std::filesystem::create_directory(path)) return path;
  }
  throw std::runtime_error("无法创建唯一联合报告目录");
}

VideoCache find_valid_cache(const OrchestratorConfig& config) {
  std::error_code error;
  std::filesystem::directory_iterator entries(config.cache_dir, error);
  if (error) throw std::runtime_error("无法扫描视频缓存目录: " + error.message());
  std::string last_error;
  for (const auto& entry : entries) {
    if (!entry.is_directory()) continue;
    try {
      return load_video_cache(entry.path(), config);
    } catch (const std::exception& invalid) {
      last_error = invalid.what();
    }
  }
  throw std::runtime_error(
      "没有与当前MCAP/config匹配且校验通过的视频manifest" +
      (last_error.empty() ? std::string{} : ": " + last_error));
}

VideoCache prepare_cache(const OrchestratorConfig& config) {
  const char* overridden_builder =
      std::getenv("JOINT_SENSOR_CACHE_BUILDER");
  const std::string builder =
      overridden_builder != nullptr && *overridden_builder != '\0'
          ? overridden_builder
          : std::string(JOINT_SENSOR_SOURCE_DIR) +
                "/tools/build_video_cache.py";
  std::vector<std::string> command{
      "/usr/bin/python3",
      builder,
      config.video_mcap.string(),
      config.cache_dir.string(),
  };
  for (const auto& lane : config.lanes) {
    command.insert(command.end(),
                   {"--lane", lane.name, lane.topic,
                    std::to_string(lane.source_fps),
                    std::to_string(lane.output_fps)});
  }
  const auto process = run_process_argv_capture_stdout(command);
  if (process.exit_code != 0) {
    throw std::runtime_error("视频缓存Python子进程失败，退出码=" +
                             std::to_string(process.exit_code));
  }
  return validate_cache_builder_output(config, process.stdout_text);
}

class RadarComponent final : public OrchestratedComponent {
 public:
  explicit RadarComponent(RadarEngine& engine) : engine_(engine) {}
  void prepare() override { engine_.prepare(); }
  void arm(std::shared_ptr<StartGate> gate) override {
    engine_.arm(std::move(gate));
  }
  void run() override { engine_.run(); }
  void wait() override {
    if (!waited_) {
      result_ = engine_.result();
      waited_ = true;
    }
  }
  void stop() noexcept override { engine_.stop(); }
  bool ok() const override { return waited_ && result_.playback.ok; }
  std::string error() const override { return result_.playback.error; }
  const RadarEngineResult& result() const { return result_; }

 private:
  RadarEngine& engine_;
  RadarEngineResult result_;
  bool waited_{};
};

class VideoComponent final : public OrchestratedComponent {
 public:
  explicit VideoComponent(VideoEngine& engine) : engine_(engine) {}
  void prepare() override { engine_.prepare(); }
  void arm(std::shared_ptr<StartGate> gate) override {
    engine_.arm(std::move(gate));
  }
  void run() override { engine_.run(); }
  void wait() override {
    if (!waited_) {
      result_ = engine_.result();
      waited_ = true;
    }
  }
  void stop() noexcept override { engine_.stop(); }
  bool ok() const override { return waited_ && result_.ok; }
  std::string error() const override { return result_.error; }
  const VideoEngineResult& result() const { return result_; }

 private:
  VideoEngine& engine_;
  VideoEngineResult result_;
  bool waited_{};
};

struct Evidence {
  std::mutex mutex;
  std::array<LaneReportV2, 3> lanes;
};

std::unique_ptr<VideoSink> make_video_sink(const OrchestratorConfig& config,
                                           const SourceWindow& window) {
  if (config.video_backend == VideoBackend::Mock) {
    MockVideoSinkOptions options;
    options.frame_bytes = kVideoFrameBytes;
    options.max_lateness_ns =
        static_cast<std::uint64_t>(config.video_max_lateness_us) * 1000ULL;
    options.output_fps = 30;
    options.grid_origin_ns = window.start_ns;
    options.play_start_ns = 0;
    for (const auto& lane : config.lanes) {
      options.lanes.push_back({lane.name, lane.source_fps, lane.h2c});
    }
    return std::make_unique<MockVideoSink>(std::move(options));
  }
  XdmaVideoSinkOptions options;
  options.allow_hardware = true;
  for (const auto& lane : config.lanes) {
    options.lane_h2c.emplace(lane.name, lane.h2c);
  }
  return std::make_unique<XdmaVideoSink>(std::move(options));
}

ReportV2Summary build_summary(const OrchestratorConfig& config,
                              const JointDatasetValidation& dataset,
                              const VideoCache& cache,
                              const OrchestratorResult& result,
                              const VideoComponent* video,
                              Evidence& evidence,
                              RadarStreamingBoundsV2 radar_bounds,
                              StopIntent stop_intent) {
  const bool xdma = config.video_backend == VideoBackend::Xdma;
  const auto finalized = finalize_run_status(
      result.ok, result.cancelled, result.error, stop_intent, xdma);
  ReportV2Summary summary;
  summary.ok = finalized.ok;
  summary.cancelled = finalized.cancelled;
  summary.backend = xdma ? "xdma" : "mock";
  summary.payload_mode = xdma ? "source_uyvy" : "synthetic";
  summary.final_state = result.lifecycle.state;
  summary.common_window = dataset.window;
  summary.play_start_ns = result.anchors.play_start_ns;
  summary.utc_anchor_us = result.anchors.utc_anchor_us;
  summary.first_failure = result.lifecycle.first_failure;
  summary.additional_failures = result.lifecycle.additional_failures;
  summary.stop_reason = finalized.stop_reason;
  summary.validation.software_logic = finalized.software_logic;
  summary.validation.video_payload_io = finalized.video_payload_io;
  summary.validation.xdma_device = finalized.xdma_device;
  summary.radar_participants = dataset.radar_topics;
  summary.video_participants = dataset.video_topics;
  summary.source_metadata_changed = cache.source_metadata_changed;
  summary.source_manifest_mtime_ns =
      cache.source_mcap_manifest_mtime_ns;
  summary.source_current_mtime_ns =
      cache.source_mcap_current_mtime_ns;
  summary.radar_streaming_bounds = radar_bounds;
  for (const auto& timing : result.timeline) {
    summary.lifecycle.push_back(
        {timing.state, timing.entered_monotonic_ns, timing.duration_ns});
  }
  {
    std::lock_guard<std::mutex> lock(evidence.mutex);
    summary.lanes = evidence.lanes;
  }
  if (video != nullptr) {
    for (std::size_t index = 0; index < summary.lanes.size(); ++index) {
      const auto& runtime = video->result().lanes[index];
      summary.lanes[index].name = runtime.name;
      summary.lanes[index].source_frames =
          runtime.source_frames_scheduled;
      summary.lanes[index].output_frames = runtime.output_frames_written;
      summary.lanes[index].queue_max_waterline =
          runtime.max_queue_waterline;
    }
  }
  return summary;
}

class PlaybackAnnouncer {
 public:
  PlaybackAnnouncer() {
    thread_ = std::jthread([this](std::stop_token stop) {
      while (!stop.stop_requested()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (stop.stop_requested()) return;
        announce_current();
      }
    });
  }

  ~PlaybackAnnouncer() { finish(); }

  void stop() {
    if (!thread_.joinable()) return;
    thread_.request_stop();
    thread_.join();
  }

  void set_phase(PlaybackPhase phase) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      phase_ = phase;
      wait_deadline_.reset();
    }
    announce_current();
  }

  void set_waiting(std::chrono::steady_clock::time_point deadline,
                   PlaybackPhase phase = PlaybackPhase::WaitingToSend) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      phase_ = phase;
      wait_deadline_ = deadline;
      sending_ = false;
    }
    announce_current();
  }

  void mark_send_start(std::chrono::milliseconds planned) {
    const auto wall = std::chrono::system_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    phase_ = PlaybackPhase::Sending;
    wait_deadline_.reset();
    sending_ = true;
    send_wall_ = wall;
    send_steady_ = std::chrono::steady_clock::now();
    planned_ = planned;
    emit(format_send_start(wall));
  }

  void finish() {
    stop();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sending_) return;
    const auto start = send_wall_;
    sending_ = false;
    emit(format_send_end(start, std::chrono::system_clock::now()));
  }

 private:
  void announce_current() {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((phase_ == PlaybackPhase::WaitingToSend ||
         phase_ == PlaybackPhase::Prefilling) &&
        wait_deadline_.has_value()) {
      auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
          *wait_deadline_ - std::chrono::steady_clock::now());
      if (left.count() < 0) left = std::chrono::milliseconds::zero();
      emit(format_phase_status(phase_, left));
      return;
    }
    if (phase_ == PlaybackPhase::Sending && sending_) {
      const auto elapsed =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now() - send_steady_);
      emit(format_phase_status(phase_, elapsed, planned_));
      return;
    }
    if (phase_ != PlaybackPhase::Sending) {
      emit(format_phase_status(phase_));
    }
  }

  static void emit(const std::string& line) {
    std::lock_guard<std::mutex> lock(output_mutex());
    std::cout << line << '\n';
    std::cout.flush();
  }

  static std::mutex& output_mutex() {
    static std::mutex mutex;
    return mutex;
  }

  std::mutex mutex_;
  std::jthread thread_;
  PlaybackPhase phase_{PlaybackPhase::CheckingData};
  std::optional<std::chrono::steady_clock::time_point> wait_deadline_;
  bool sending_{false};
  std::chrono::system_clock::time_point send_wall_{};
  std::chrono::steady_clock::time_point send_steady_{};
  std::chrono::milliseconds planned_{};
};

}  // namespace

// main 的顺序，后面的注释按这个往下对：
// 1. 读配置。--prepare-cache 只生成视频缓存就退出。
// 2. 核对已有的 UYVY 缓存和八路 lidar 的轻量目录，得到共同窗口。
//    本文件里的 radar 名字就是这八路 lidar，没有另一套毫米波雷达。
// 3. 先建报告目录。此后失败也会留下 summary.json。
// 4. --dry-run 只数窗口里有多少帧，不打开设备、不发 UDP。
// 5. 视频要么整段进 /dev/shm，要么留在 SSD、只在内存里留滑动窗口。
// 6. mlock 点云，选核，建 lidar 引擎和视频引擎。线程还没开始跑。
// 7. 若配置了环视，先把四路排期帧读进内存。这一步在 run() 之前。
// 8. orchestrator.run()：准备设备、7 个线程到齐、倒计时、等回车、
//    放出 play_start、预热队列、按绝对期限发送，直到两边结束。
// 9. 收环视结果，写 summary。视频失败返回 3，仅环视失败返回 4。
int main(int argc, char** argv) {
  std::signal(SIGINT, handle_stop_signal);
  std::signal(SIGTERM, handle_stop_signal);
  try {
    const auto arguments = parse_arguments(argc, argv);
    const auto config = load_orchestrator_config(arguments.config);
    // 解码 H.265 并写出 UYVY 清单。不进入发送。
    if (arguments.prepare_cache) {
      const auto cache = prepare_cache(config);
      std::cout << "CACHE_READY dataset=" << cache.dataset_id
                << " key=" << cache.cache_key << '\n';
      return 0;
    }

    // 视频缓存必须已经在 cache_dir 里，并且和这次的车道配置一致。
    auto cache = find_valid_cache(config);
    // 只扫每帧的时间和包数。UDP 载荷留到发送线程里再读。
    auto radar_catalog =
        joint_sensor_player::scan_mcap_directory_lightweight(
        config.lidar.data_dir, config.lidar.channels);
    // 共同窗口是视频时间戳和八路 lidar 帧时间的交集。槽和包的期限都相对它。
    const auto dataset =
        validate_joint_dataset(config, cache, radar_catalog);
    const StreamingRadarLimits streaming_limits;
    const auto radar_preflight = analyze_streaming_radar(
        config.lidar, radar_catalog, dataset.window);
    const RadarStreamingBoundsV2 radar_bounds{
        radar_preflight.required_frame_reorder,
        streaming_limits.frame_reorder_capacity,
        radar_preflight.required_packet_buffer,
        streaming_limits.packet_queue_capacity};
    // 报告目录一建立，ReportFailureGuard 就保证中途异常也会写出 summary.json。
    const auto run_dir = create_run_directory(config.lidar.report_dir);
    ReportV2 report(run_dir);
    ReportV2Summary failure_summary;
    failure_summary.backend =
        config.video_backend == VideoBackend::Mock ? "mock" : "xdma";
    failure_summary.payload_mode =
        config.video_backend == VideoBackend::Mock ? "synthetic"
                                                  : "source_uyvy";
    failure_summary.final_state = LifecycleState::Failed;
    failure_summary.common_window = dataset.window;
    failure_summary.radar_participants = dataset.radar_topics;
    failure_summary.video_participants = dataset.video_topics;
    failure_summary.source_metadata_changed = cache.source_metadata_changed;
    failure_summary.source_manifest_mtime_ns =
        cache.source_mcap_manifest_mtime_ns;
    failure_summary.source_current_mtime_ns =
        cache.source_mcap_current_mtime_ns;
    failure_summary.radar_streaming_bounds = radar_bounds;
    ReportFailureGuard report_guard(report, failure_summary);
    try {
      std::filesystem::copy_file(
          arguments.config, run_dir / "config_snapshot.yaml",
          std::filesystem::copy_options::overwrite_existing);
      if (cache.source_metadata_changed) {
        report.log(
            "source_metadata_changed=true manifest_mtime_ns=" +
            std::to_string(cache.source_mcap_manifest_mtime_ns) +
            " current_mtime_ns=" +
            std::to_string(cache.source_mcap_current_mtime_ns));
      }

      // 干跑：用 play_start=0 数每路会发出多少张不重复的源帧，然后退出。
      if (arguments.dry_run) {
        ReportV2Summary summary = failure_summary;
        summary.ok = true;
        summary.final_state = LifecycleState::Stopped;
        summary.cancelled = g_signal_stop != 0;
        summary.stop_reason = summary.cancelled ? "signal" : "dry_run";
        summary.validation.software_logic = ValidationStatus::NotRun;
        for (std::size_t index = 0; index < cache.lanes.size(); ++index) {
          summary.lanes[index].name = cache.lanes[index].name;
          const auto slots = build_video_slots(
              cache.lanes[index].timestamps_ns,
              cache.lanes[index].source_fps,
              cache.lanes[index].output_fps, dataset.window, 0);
          std::optional<std::uint64_t> previous_source;
          for (const auto& slot : slots) {
            if (!previous_source ||
                *previous_source != slot.source_index) {
              ++summary.lanes[index].source_frames;
              previous_source = slot.source_index;
            }
          }
        }
        report_guard.complete(summary);
        std::cout << (summary.cancelled ? "STOPPED(signal)" : "DRY_RUN_OK")
                  << " report=" << run_dir << '\n';
        return 0;
      }

      // 从此以后的失败记成一次真实播放失败，而不是还没开播的检查失败。
      report_guard.mark_execution_attempted();
      // 终端每秒打印一行「状态:」。真正的发送还没开始。
      PlaybackAnnouncer announcer;
      announcer.set_phase(config.video_backend == VideoBackend::Xdma
                              ? PlaybackPhase::PreparingVideo
                              : PlaybackPhase::PreparingRadar);
      // 尝试把进程 nice 降到 -5，减轻大块 UYVY 预读对 lidar 发送的挤占。
      // 没有权限就保持原优先级，播放继续，不依赖这一步。
      if (setpriority(PRIO_PROCESS, 0, -5) == 0) {
        report.log("process_nice=-5");
      } else {
        report.log("process_nice_unchanged");
      }
      // 两条视频内存路径只走一条。
      // video_prefetch_ms > 0：UYVY 留在 SSD，发送时由 filler 维护滑动窗口。
      // video_prefetch_ms == 0：把共同窗口整段拷进 /dev/shm，后面从内存盘读。
      VideoPlaybackPin video_pin;
      if (config.video_backend == VideoBackend::Xdma &&
          config.video_prefetch_ms > 0) {
        report.log("video_prefetch_ms=" +
                   std::to_string(config.video_prefetch_ms) +
                   " staged_bytes=0");
      } else if (config.video_backend == VideoBackend::Xdma) {
        std::optional<std::uint64_t> relative;
        if (config.play_duration_ms > 0) {
          relative = config.play_duration_ms * 1'000'000ULL +
                     1'000'000'000ULL;
        }
        VideoPrefaultOptions prefault_options;
        prefault_options.stage_to_shm = true;
        prefault_options.drop_outside_pages = true;
        // 整段视频不 mlock。锁定额度大约 3.8 GiB，留给下面的点云。
        // 把数 GiB 视频也锁进内存时，lidar UDP 会被挤过门限。
        prefault_options.max_lock_bytes = 0;
        video_pin = prefault_and_pin_video_playback_window(
            cache, dataset.window, relative, prefault_options);
        video_pin.apply_staged_payload_paths(cache);
        report.log(
            "video_prefault_frames=" +
            std::to_string(video_pin.frames_prefaulted()) +
            " staged_bytes=" +
            std::to_string(video_pin.staged_bytes()) +
            " locked_bytes=" +
            std::to_string(video_pin.locked_bytes()) +
            " dontneed_bytes=" +
            std::to_string(video_pin.outside_bytes_dontneed()));
      }
      // 点云大约 1.1 GiB。钉住它，避免播放中途被换出而耽误包的期限。
      // 只播视频的冒烟没有 lidar 目录，跳过这一步。
      if (config.video_backend == VideoBackend::Xdma) {
        announcer.set_phase(PlaybackPhase::PreparingRadar);
      }
      PinnedFiles radar_pin;
      if (!config.allow_video_only_smoke) {
        radar_pin = pin_regular_files(
            config.lidar.data_dir,
            std::numeric_limits<std::uint64_t>::max());
        report.log("radar_pin_files=" +
                   std::to_string(radar_pin.file_count()) +
                   " locked_bytes=" +
                   std::to_string(radar_pin.locked_bytes()));
      }
      // 生命周期和单调钟是视频、lidar 共用的。线程要到 orchestrator.run() 才启动。
      LifecycleManager lifecycle;
      joint_sensor_player::MonotonicClock clock;
      // 视频写卡、生产和预读各占一个物理核。lidar 发送和读取保持 -1，不绑核。
      const auto affinity =
          plan_send_affinity(online_physical_cores());
      if (affinity.enabled) {
        std::ostringstream affinity_line;
        affinity_line << "thread_affinity consumers="
                      << affinity.video_consumers[0] << ','
                      << affinity.video_consumers[1] << ','
                      << affinity.video_consumers[2]
                      << " producers=" << affinity.video_producers[0] << ','
                      << affinity.video_producers[1] << ','
                      << affinity.video_producers[2]
                      << " filler=" << affinity.filler
                      << " radar_sender=" << affinity.radar_sender
                      << " radar_reader=" << affinity.radar_reader
                      << " shadow=" << affinity.shadow[0] << ','
                      << affinity.shadow[1] << ',' << affinity.shadow[2]
                      << ',' << affinity.shadow[3];
        report.log(affinity_line.str());
      } else {
        report.log("thread_affinity_disabled");
      }
      // lidar 侧在 StartGate 里只占 1 个名额。
      // 冒烟时用一个空组件占住这个名额，不创建 UDP socket。
      std::unique_ptr<joint_sensor_player::UdpDatagramSink> udp;
      std::unique_ptr<RadarEngine> radar_engine;
      std::unique_ptr<OrchestratedComponent> radar;
      if (config.allow_video_only_smoke) {
        radar = std::make_unique<SmokeRadarComponent>(lifecycle);
      } else {
        udp = std::make_unique<joint_sensor_player::UdpDatagramSink>(
            config.lidar);
        RadarEngineDependencies radar_dependencies;
        radar_dependencies.clock = &clock;
        radar_dependencies.sink = udp.get();
        radar_dependencies.prevalidated_lightweight_catalog =
            std::move(radar_catalog);
        radar_dependencies.sender_cpu = affinity.radar_sender;
        radar_dependencies.reader_cpu = affinity.radar_reader;
        radar_dependencies.frame_observer =
            [&](const joint_sensor_player::FrameStat& stat) {
              std::ostringstream row;
              row << stat.slot << ',' << stat.topic << ','
                  << stat.source_timestamp_ns << ','
                  << stat.planned_last_packet_ns << ','
                  << stat.actual_last_packet_ns << ','
                  << stat.packet_count << ',' << stat.lateness_ns;
              report.record_radar_frame(row.str());
            };
        radar_engine = std::make_unique<RadarEngine>(
            config.lidar, dataset.window, lifecycle,
            std::move(radar_dependencies));
        radar = std::make_unique<RadarComponent>(*radar_engine);
      }
      // Mock 不打开 /dev/xdma。XDMA 要等 run() 里的 prepare 才真正 open 三个 H2C。
      auto video_sink = make_video_sink(config, dataset.window);
      Evidence evidence;
      for (std::size_t index = 0; index < config.lanes.size(); ++index) {
        evidence.lanes[index].name = config.lanes[index].name;
      }
      VideoEngineDependencies video_dependencies;
      video_dependencies.consumer_cpu = affinity.video_consumers;
      video_dependencies.producer_cpu = affinity.video_producers;
      video_dependencies.filler_cpu = affinity.filler;
      video_dependencies.clock = &clock;
      video_dependencies.sink = video_sink.get();
      video_dependencies.frame_observer =
          [&](const std::string& lane, const VideoSlot& slot,
              std::uint64_t actual, std::uint64_t lateness) {
            report.record_video_frame(
                {lane, slot.source_index, slot.output_index,
                 slot.source_time_ns, slot.slot_time_ns, slot.deadline_ns,
                 actual, lateness});
            std::lock_guard<std::mutex> lock(evidence.mutex);
            const auto found = std::find_if(
                evidence.lanes.begin(), evidence.lanes.end(),
                [&](const auto& item) { return item.name == lane; });
            if (found == evidence.lanes.end()) return;
            if (found->lateness_samples == 0 ||
                lateness < found->lateness_min_ns) {
              found->lateness_min_ns = lateness;
            }
            found->lateness_max_ns =
                std::max(found->lateness_max_ns, lateness);
            found->lateness_sum_ns += lateness;
            found->late_frames += lateness > 0 ? 1 : 0;
            ++found->lateness_samples;
          };
      // 这里只建对象。video_prefetch_ms > 0 时滑动窗口也在构造函数里建好，filler 尚未运行。
      VideoEngine video_engine(
          config, cache, dataset.window, lifecycle,
          std::move(video_dependencies));
      VideoComponent video(video_engine);

      // Ctrl+C 只置停止开关。报告里据此区分 signal 和正常播完。
      std::atomic<StopIntent> stop_intent{StopIntent::None};
      std::jthread signal_monitor(
          [&](std::stop_token stopped) {
            while (!stopped.stop_requested()) {
              if (g_signal_stop != 0) {
                StopIntent expected = StopIntent::None;
                stop_intent.compare_exchange_strong(
                    expected, StopIntent::Signal);
                lifecycle.request_stop();
                return;
              }
              std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
          });
      OrchestratorOptions options;
      // start_delay_ms 在 wait_for_enter 时用两次，都还没开始写卡：
      // 先在门齐之后倒计时并等回车；回车后 play_start 再往后推这么久，用来填队列。
      // 倒计时发生在放出锚点之前，不能把它算进 play_start，否则线程还在读帧，期限已经到了。
      options.start_delay_ns =
          static_cast<std::uint64_t>(config.lidar.start_delay_ms) *
          1'000'000ULL;
      // before_release 在 7 个线程已经到达 StartGate、锚点尚未发出时调用。
      // 此时 producer 还没拿到 play_start，队列仍是空的。filler 若存在，已经在读盘。
      std::jthread duration_monitor;
      options.before_release = [&] {
        // 不等回车：倒计时结束就进入预热，play_duration_ms > 0 时到点停止。
        if (!config.wait_for_enter) {
          duration_monitor = std::jthread(
              [&](std::stop_token stopped) {
                // 播放时长从 play_start 起算，不从门打开起算。
                // start_delay 只用于预热，不占用 play_duration_ms。
                const auto play_start_wall =
                    std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(config.lidar.start_delay_ms);
                announcer.set_waiting(play_start_wall);
                while (!stopped.stop_requested() &&
                       std::chrono::steady_clock::now() < play_start_wall) {
                  std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                if (stopped.stop_requested()) return;
                announcer.mark_send_start(
                    std::chrono::milliseconds(config.play_duration_ms));
                if (config.play_duration_ms == 0) return;
                const auto deadline =
                    play_start_wall +
                    std::chrono::milliseconds(config.play_duration_ms);
                while (!stopped.stop_requested() &&
                       std::chrono::steady_clock::now() < deadline) {
                  std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                if (stopped.stop_requested()) return;
                StopIntent expected = StopIntent::None;
                if (stop_intent.compare_exchange_strong(
                        expected, StopIntent::DurationLimit)) {
                  lifecycle.request_stop();
                }
              });
          return;
        }
        // 等回车：先倒计时 start_delay_ms，再丢掉倒计时期间敲进终端的换行。
        const auto countdown_done =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(config.lidar.start_delay_ms);
        announcer.set_waiting(countdown_done);
        while (!lifecycle.stop_token()->stop_requested() &&
               g_signal_stop == 0 &&
               std::chrono::steady_clock::now() < countdown_done) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (lifecycle.stop_token()->stop_requested() || g_signal_stop != 0) {
          return;
        }
        // 倒计时期间按下的回车留在终端缓冲里，提示一出来就会被当成确认。
        // 先丢掉这些输入，再打印等待，只有此后新的回车才进入预热。
        discard_pending_input(STDIN_FILENO);
        announcer.set_phase(PlaybackPhase::WaitingForEnter);
        const auto entered = wait_for_newline(STDIN_FILENO, [&] {
          return lifecycle.stop_token()->stop_requested() ||
                 g_signal_stop != 0;
        });
        if (entered != EnterWaitResult::Confirmed) {
          if (entered == EnterWaitResult::Closed) lifecycle.request_stop();
          return;
        }
        // 新的回车之后再等一个 start_delay_ms。这段是队列预热，终端显示「尚未发送」。
        duration_monitor = std::jthread(
            [&](std::stop_token stopped) {
              const auto play_start_wall =
                  std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(config.lidar.start_delay_ms);
              announcer.set_waiting(play_start_wall,
                                    PlaybackPhase::Prefilling);
              while (!stopped.stop_requested() &&
                     std::chrono::steady_clock::now() < play_start_wall) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
              }
              if (stopped.stop_requested()) return;
              announcer.mark_send_start(
                  std::chrono::milliseconds(config.play_duration_ms));
              if (config.play_duration_ms == 0) return;
              const auto deadline =
                  play_start_wall +
                  std::chrono::milliseconds(config.play_duration_ms);
              while (!stopped.stop_requested() &&
                     std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
              }
              if (stopped.stop_requested()) return;
              StopIntent expected = StopIntent::None;
              if (stop_intent.compare_exchange_strong(
                      expected, StopIntent::DurationLimit)) {
                lifecycle.request_stop();
              }
            });
      };
      // 环视不参加 StartGate。排期帧必须在 run() 之前读完，发送期间不再读盘。
      std::unique_ptr<ShadowVideo> shadow;
      if (!config.shadow_manifest.empty()) {
        if (config.video_prefetch_ms == 0) {
          throw std::runtime_error("环视模拟需要video_prefetch_ms");
        }
        shadow = std::make_unique<ShadowVideo>(
            load_shadow_lanes(config.shadow_manifest), dataset.window, clock,
            lifecycle.stop_token(),
            static_cast<std::uint64_t>(config.video_prefetch_ms) * 1'000'000ULL,
            affinity.shadow);
        shadow->start();
        if (!shadow->wait_until_preloaded()) {
          throw std::runtime_error("环视模拟预读失败");
        }
        report.log("shadow_before_h2c preloaded manifest=" +
                   config.shadow_manifest.string());
      }
      // 锚点放出之后立刻通知环视。它不参加那 7 个线程的门。
      options.on_play_start = [&](const StartAnchors& anchors) {
        if (shadow) shadow->notify_play_start(anchors);
      };
      // run() 内部才打开 H2C、拉起线程、调用上面的 before_release，然后等待发送结束。
      Orchestrator orchestrator(
          lifecycle, *radar, video, std::move(options));
      const auto result = orchestrator.run();
      ShadowVideoResult shadow_result;
      if (shadow) {
        shadow_result = shadow->wait();
        std::ostringstream shadow_line;
        shadow_line << "shadow_before_h2c ok=" << shadow_result.ok;
        for (const auto& lane : shadow_result.lanes) {
          shadow_line << ' ' << lane.name << " read=" << lane.frames_read
                      << " outputs=" << lane.outputs
                      << " max_lateness_us=" << (lane.max_lateness_ns / 1000)
                      << " h2c_writes=" << lane.h2c_writes;
        }
        if (!shadow_result.error.empty()) {
          shadow_line << " error=" << shadow_result.error;
        }
        report.log(shadow_line.str());
      }
      signal_monitor.request_stop();
      if (duration_monitor.joinable()) duration_monitor.request_stop();
      announcer.finish();
      // 视频和 lidar 的引擎结果进 summary。环视失败单独看，不改写引擎的 ok。
      const auto summary = build_summary(
          config, dataset, cache, result, &video, evidence, radar_bounds,
          stop_intent.load());
      report_guard.complete(summary);
      // 3：视频或 lidar 没过门限。4：这两边过了，环视自己失败。0：播完、到时或信号停止。
      if (!summary.ok) {
        std::cerr << "联合播放失败: " << summary.stop_reason << '\n';
        if (shadow && !shadow_result.ok) {
          std::cerr << "环视模拟失败: " << shadow_result.error << '\n';
        }
        return 3;
      }
      if (shadow && !shadow_result.ok) {
        std::cerr << "环视模拟失败: " << shadow_result.error << '\n';
        return 4;
      }
      if (summary.cancelled) {
        std::cout << "STOPPED(signal) report=" << run_dir << '\n';
      } else if (summary.stop_reason == "duration_limit") {
        std::cout << "STOPPED(duration_limit) report=" << run_dir << '\n';
      } else {
        std::cout << "STOPPED(completed) report=" << run_dir << '\n';
      }
      return 0;
    } catch (const std::exception& error) {
      report_guard.set_failure("main", error.what());
      throw;
    }
  } catch (const std::exception& error) {
    std::cerr << "错误: " << error.what() << '\n';
    usage(std::cerr);
    return 2;
  }
}
