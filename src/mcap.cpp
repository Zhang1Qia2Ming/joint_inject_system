#include "joint_sensor_player/mcap.hpp"
#include "joint_sensor_player/packet_time_rewriter.hpp"

#include <openssl/evp.h>
#include <zstd.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <queue>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace joint_sensor_player {
namespace {

constexpr std::array<std::uint8_t, 8> kMagic{
    0x89, 'M', 'C', 'A', 'P', '0', '\r', '\n'};
constexpr std::uint8_t kChannel = 0x04;
constexpr std::uint8_t kMessage = 0x05;
constexpr std::uint8_t kChunk = 0x06;
constexpr std::uint64_t kMaxRecordSize = 512ULL * 1024 * 1024;
constexpr std::uint64_t kMaxStreamingChunkSize =
    64ULL * 1024 * 1024;
using FrameCallback =
    std::function<void(const std::string&, LidarFrame&&)>;

std::runtime_error chunk_limit_error(std::uint64_t limit) {
  if (limit == kMaxStreamingChunkSize) {
    return std::runtime_error(
        "MCAP流式Chunk超过64 MiB硬上限");
  }
  return std::runtime_error("MCAP Chunk长度非法");
}

std::runtime_error record_limit_error(std::uint64_t limit) {
  if (limit == kMaxStreamingChunkSize) {
    return std::runtime_error(
        "MCAP流式顶层记录超过64 MiB硬上限");
  }
  return std::runtime_error("MCAP记录过大或尾部损坏");
}

template <typename T>
T load_le(const std::uint8_t* data) {
  T value{};
  std::memcpy(&value, data, sizeof(value));
  return value;
}

class BufferReader {
 public:
  BufferReader(const std::uint8_t* data, std::size_t size)
      : data_(data), size_(size) {}

  template <typename T>
  T pod() {
    require(sizeof(T));
    const T value = load_le<T>(data_ + offset_);
    offset_ += sizeof(T);
    return value;
  }

  std::string string() {
    const auto size = pod<std::uint32_t>();
    require(size);
    std::string value(reinterpret_cast<const char*>(data_ + offset_), size);
    offset_ += size;
    return value;
  }

  const std::uint8_t* bytes(std::size_t size) {
    require(size);
    const auto* value = data_ + offset_;
    offset_ += size;
    return value;
  }

  std::size_t remaining() const { return size_ - offset_; }

 private:
  void require(std::size_t size) {
    if (size > remaining()) throw std::runtime_error("MCAP记录越界");
  }

  const std::uint8_t* data_;
  std::size_t size_;
  std::size_t offset_{};
};

struct RecordHeader {
  std::uint8_t opcode{};
  std::uint64_t length{};
};

bool read_record_header(std::ifstream& input, std::uint64_t data_end,
                        RecordHeader& header,
                        std::uint64_t max_record_size) {
  const auto position = input.tellg();
  if (position < 0 || static_cast<std::uint64_t>(position) + 9 > data_end) {
    return false;
  }
  char opcode{};
  if (!input.read(&opcode, 1)) return false;
  header.opcode = static_cast<std::uint8_t>(opcode);
  if (!input.read(reinterpret_cast<char*>(&header.length),
                  sizeof(header.length))) {
    return false;
  }
  if (header.length > max_record_size) {
    throw record_limit_error(max_record_size);
  }
  const auto body_position = input.tellg();
  if (body_position < 0 ||
      header.length >
          data_end - static_cast<std::uint64_t>(body_position)) {
    throw std::runtime_error("MCAP顶层记录越过文件边界");
  }
  return true;
}

std::vector<std::uint8_t> read_bytes(std::ifstream& input, std::size_t size) {
  std::vector<std::uint8_t> bytes(size);
  if (size != 0 &&
      !input.read(reinterpret_cast<char*>(bytes.data()),
                  static_cast<std::streamsize>(size))) {
    throw std::runtime_error("读取MCAP记录失败");
  }
  return bytes;
}

void parse_channel(const std::uint8_t* data, std::size_t size,
                   std::unordered_map<std::uint16_t, std::string>& channels) {
  BufferReader reader(data, size);
  const auto id = reader.pod<std::uint16_t>();
  (void)reader.pod<std::uint16_t>();
  const auto topic = reader.string();
  const auto encoding = reader.string();
  if (encoding != "cdr") {
    throw std::runtime_error("雷达Channel不是cdr编码: " + topic);
  }
  channels[id] = topic;
}

LidarFrame decode_frame(const std::uint8_t* data, std::size_t size,
                        const LidarChannelConfig& channel,
                        std::uint64_t log_time, std::uint32_t sequence,
                        const std::filesystem::path& source) {
  if (size < 48) throw std::runtime_error("UdpFrame CDR头长度不足");
  if (data[0] != 0 || data[1] != 1 || data[2] != 0 || data[3] != 0) {
    throw std::runtime_error("UdpFrame CDR封装头不匹配");
  }
  const auto sec = load_le<std::int32_t>(data + 4);
  const auto nsec = load_le<std::uint32_t>(data + 8);
  if (sec < 0 || nsec >= 1000000000U) {
    throw std::runtime_error("UdpFrame时间戳非法");
  }
  const auto count = load_le<std::uint32_t>(data + 44);
  if (count == 0 || count > channel.packets_per_frame) {
    throw std::runtime_error("UdpFrame包数量非法");
  }
  const std::size_t padding =
      (4 - ((8 + channel.packet_size) % 4)) % 4;
  const std::size_t stride = 8 + channel.packet_size + padding + 4;
  if (count > (std::numeric_limits<std::size_t>::max() - 48) / stride ||
      48 + static_cast<std::size_t>(count) * stride > size) {
    throw std::runtime_error("UdpFrame数据长度不足");
  }

  LidarFrame frame;
  frame.source_timestamp_ns =
      static_cast<std::uint64_t>(sec) * 1000000000ULL + nsec;
  frame.bag_log_time_ns = log_time;
  frame.sequence = sequence;
  frame.source_file = source;
  frame.packets.reserve(count);
  for (std::uint32_t index = 0; index < count; ++index) {
    const auto* packet = data + 48 + static_cast<std::size_t>(index) * stride;
    const auto payload_size =
        load_le<std::uint32_t>(packet + 8 + channel.packet_size + padding);
    if (payload_size == 0 || payload_size > channel.packet_size) {
      throw std::runtime_error("UdpPacket有效长度非法");
    }
    frame.packets.emplace_back(packet + 8, packet + 8 + payload_size);
  }
  return frame;
}

void parse_message(
    const std::uint8_t* data, std::size_t size,
    const std::unordered_map<std::uint16_t, std::string>& channel_topics,
    const std::unordered_map<std::string, LidarChannelConfig>& wanted,
    const std::filesystem::path& source, const FrameCallback& callback) {
  if (size < 22) throw std::runtime_error("MCAP Message记录过短");
  BufferReader reader(data, size);
  const auto channel_id = reader.pod<std::uint16_t>();
  const auto sequence = reader.pod<std::uint32_t>();
  const auto log_time = reader.pod<std::uint64_t>();
  (void)reader.pod<std::uint64_t>();
  const auto topic_it = channel_topics.find(channel_id);
  if (topic_it == channel_topics.end()) return;
  const auto wanted_it = wanted.find(topic_it->second);
  if (wanted_it == wanted.end()) return;
  const auto payload_size = reader.remaining();
  const auto* payload = reader.bytes(payload_size);
  auto frame = decode_frame(payload, payload_size, wanted_it->second, log_time,
                            sequence, source);
  callback(topic_it->second, std::move(frame));
}

void parse_record_buffer(
    const std::uint8_t* data, std::size_t size,
    std::unordered_map<std::uint16_t, std::string>& channel_topics,
    const std::unordered_map<std::string, LidarChannelConfig>& wanted,
    const std::filesystem::path& source, const FrameCallback& callback) {
  std::size_t offset = 0;
  while (offset + 9 <= size) {
    const auto opcode = data[offset];
    const auto length = load_le<std::uint64_t>(data + offset + 1);
    offset += 9;
    if (length > size - offset) throw std::runtime_error("Chunk内部记录越界");
    if (opcode == kChannel) {
      parse_channel(data + offset, static_cast<std::size_t>(length),
                    channel_topics);
    } else if (opcode == kMessage) {
      parse_message(data + offset, static_cast<std::size_t>(length),
                    channel_topics, wanted, source, callback);
    }
    offset += static_cast<std::size_t>(length);
  }
  if (offset != size) throw std::runtime_error("Chunk尾部存在残缺记录");
}

void parse_chunk(
    const std::vector<std::uint8_t>& body,
    std::unordered_map<std::uint16_t, std::string>& channel_topics,
    const std::unordered_map<std::string, LidarChannelConfig>& wanted,
    const std::filesystem::path& source, const FrameCallback& callback,
    std::size_t* peak_chunk_bytes, std::uint64_t max_chunk_size) {
  BufferReader reader(body.data(), body.size());
  (void)reader.pod<std::uint64_t>();
  (void)reader.pod<std::uint64_t>();
  const auto uncompressed_size = reader.pod<std::uint64_t>();
  (void)reader.pod<std::uint32_t>();
  const auto compression = reader.string();
  const auto compressed_size = reader.pod<std::uint64_t>();
  if (uncompressed_size > max_chunk_size ||
      compressed_size > max_chunk_size) {
    throw chunk_limit_error(max_chunk_size);
  }
  if (compressed_size > reader.remaining()) {
    throw std::runtime_error("MCAP Chunk长度非法");
  }
  const auto* compressed = reader.bytes(static_cast<std::size_t>(compressed_size));
  std::vector<std::uint8_t> raw(static_cast<std::size_t>(uncompressed_size));
  if (peak_chunk_bytes != nullptr) {
    *peak_chunk_bytes = std::max(*peak_chunk_bytes, raw.size());
  }
  if (compression.empty()) {
    if (compressed_size != uncompressed_size) {
      throw std::runtime_error("未压缩Chunk长度不一致");
    }
    std::copy(compressed, compressed + compressed_size, raw.begin());
  } else if (compression == "zstd") {
    const auto result = ZSTD_decompress(raw.data(), raw.size(), compressed,
                                        static_cast<std::size_t>(compressed_size));
    if (ZSTD_isError(result) || result != raw.size()) {
      throw std::runtime_error("MCAP zstd解压失败");
    }
  } else {
    throw std::runtime_error("不支持的MCAP压缩: " + compression);
  }
  parse_record_buffer(raw.data(), raw.size(), channel_topics, wanted, source,
                      callback);
}

void parse_file(
    const std::filesystem::path& path,
    const std::unordered_map<std::string, LidarChannelConfig>& wanted,
    const FrameCallback& callback, std::size_t* peak_chunk_bytes = nullptr,
    std::uint64_t max_record_size = kMaxRecordSize) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("无法打开MCAP: " + path.string());
  input.seekg(0, std::ios::end);
  const auto file_size_pos = input.tellg();
  if (file_size_pos < 16) throw std::runtime_error("MCAP文件过短");
  const auto file_size = static_cast<std::uint64_t>(file_size_pos);
  const auto records_end = file_size - kMagic.size();
  input.seekg(0);
  std::array<std::uint8_t, 8> magic{};
  input.read(reinterpret_cast<char*>(magic.data()), magic.size());
  if (magic != kMagic) throw std::runtime_error("MCAP magic不匹配");
  input.seekg(static_cast<std::streamoff>(records_end));
  std::array<std::uint8_t, 8> tail_magic{};
  input.read(reinterpret_cast<char*>(tail_magic.data()), tail_magic.size());
  if (tail_magic != kMagic) throw std::runtime_error("MCAP尾部magic不匹配");
  input.clear();
  input.seekg(kMagic.size());

  std::unordered_map<std::uint16_t, std::string> channel_topics;
  while (input && static_cast<std::uint64_t>(input.tellg()) < records_end) {
    RecordHeader header;
    if (!read_record_header(
            input, records_end, header, max_record_size)) {
      break;
    }
    if (header.opcode == kChannel) {
      const auto body = read_bytes(input, static_cast<std::size_t>(header.length));
      parse_channel(body.data(), body.size(), channel_topics);
    } else {
      input.seekg(static_cast<std::streamoff>(header.length), std::ios::cur);
    }
  }

  input.clear();
  input.seekg(kMagic.size());
  while (input && static_cast<std::uint64_t>(input.tellg()) < records_end) {
    RecordHeader header;
    if (!read_record_header(
            input, records_end, header, max_record_size)) {
      break;
    }
    if (header.opcode == kMessage || header.opcode == kChunk ||
        header.opcode == kChannel) {
      const auto body = read_bytes(input, static_cast<std::size_t>(header.length));
      if (header.opcode == kMessage) {
        parse_message(body.data(), body.size(), channel_topics, wanted, path,
                      callback);
      } else if (header.opcode == kChunk) {
        parse_chunk(body, channel_topics, wanted, path, callback,
                    peak_chunk_bytes, max_record_size);
      }
    } else {
      input.seekg(static_cast<std::streamoff>(header.length), std::ios::cur);
    }
  }
}

bool same_payload(const LidarFrame& left, const LidarFrame& right) {
  return left.packets == right.packets;
}

LidarFrameMetadata summarize_frame(const LidarFrame& frame) {
  if (frame.packets.empty()) {
    throw std::runtime_error("雷达帧不包含UDP包");
  }
  LidarFrameMetadata result;
  result.source_timestamp_ns = frame.source_timestamp_ns;
  result.bag_log_time_ns = frame.bag_log_time_ns;
  result.sequence = frame.sequence;
  result.packet_count = static_cast<std::uint32_t>(frame.packets.size());
  using DigestContext =
      std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
  DigestContext digest(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!digest ||
      EVP_DigestInit_ex(digest.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("雷达payload摘要初始化失败");
  }
  std::optional<LidarPacketProtocol> protocol;
  std::optional<std::uint64_t> previous_time;
  std::optional<std::uint32_t> previous_sequence;
  for (const auto& packet : frame.packets) {
    const auto info = inspect_lidar_packet(packet);
    if (protocol && *protocol != info.protocol) {
      throw std::runtime_error("同一雷达帧混入不同UDP协议");
    }
    if (previous_time && info.sensor_time_us <= *previous_time) {
      throw std::runtime_error("雷达包内时间未严格递增");
    }
    if (previous_sequence) {
      const std::uint32_t delta = info.sequence - *previous_sequence;
      if (delta == 0 || delta >= 0x80000000U) {
        throw std::runtime_error("雷达包内序号倒退或异常跳变");
      }
    }
    if (!previous_time) {
      result.first_packet_time_us = info.sensor_time_us;
      result.first_packet_sequence = info.sequence;
    }
    previous_time = info.sensor_time_us;
    previous_sequence = info.sequence;
    protocol = info.protocol;
    result.payload_bytes += packet.size();
    const auto size = static_cast<std::uint64_t>(packet.size());
    if (EVP_DigestUpdate(digest.get(), &size, sizeof(size)) != 1 ||
        EVP_DigestUpdate(
            digest.get(), packet.data(), packet.size()) != 1) {
      throw std::runtime_error("雷达payload摘要更新失败");
    }
  }
  result.last_packet_time_us = *previous_time;
  result.last_packet_sequence = *previous_sequence;
  result.protocol = static_cast<std::uint8_t>(*protocol);
  unsigned digest_size{};
  if (EVP_DigestFinal_ex(
          digest.get(), result.payload_sha256.data(),
          &digest_size) != 1 ||
      digest_size != result.payload_sha256.size()) {
    throw std::runtime_error("雷达payload摘要计算失败");
  }
  return result;
}

std::vector<std::filesystem::path> list_mcap_files(
    const std::filesystem::path& directory) {
  if (!std::filesystem::is_directory(directory)) {
    throw std::runtime_error("MCAP目录不存在: " + directory.string());
  }
  std::vector<std::filesystem::path> files;
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    if (entry.is_regular_file() && entry.path().extension() == ".mcap") {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  if (files.empty()) {
    throw std::runtime_error("data目录第一层没有MCAP文件");
  }
  return files;
}

}  // namespace

McapCatalog scan_mcap_directory(
    const std::filesystem::path& directory,
    const std::vector<LidarChannelConfig>& channels) {
  if (!std::filesystem::is_directory(directory)) {
    throw std::runtime_error("MCAP目录不存在: " + directory.string());
  }
  std::unordered_map<std::string, LidarChannelConfig> wanted;
  for (const auto& channel : channels) {
    if (channel.enabled) wanted.emplace(channel.topic, channel);
  }

  McapCatalog catalog;
  catalog.files = list_mcap_files(directory);
  const FrameCallback retain =
      [&catalog](const std::string& topic, LidarFrame&& frame) {
        catalog.frames[topic].push_back(std::move(frame));
      };
  for (const auto& path : catalog.files) {
    parse_file(path, wanted, retain);
  }

  for (auto& [topic, frames] : catalog.frames) {
    std::stable_sort(frames.begin(), frames.end(),
                     [](const auto& left, const auto& right) {
                       if (left.source_timestamp_ns != right.source_timestamp_ns) {
                         return left.source_timestamp_ns < right.source_timestamp_ns;
                       }
                       return left.bag_log_time_ns < right.bag_log_time_ns;
                     });
    std::vector<LidarFrame> merged;
    merged.reserve(frames.size());
    for (auto& frame : frames) {
      if (!merged.empty() &&
          merged.back().source_timestamp_ns == frame.source_timestamp_ns) {
        if (!same_payload(merged.back(), frame)) {
          throw std::runtime_error("同一topic同一时间戳内容冲突: " + topic);
        }
        ++catalog.duplicate_messages;
        continue;
      }
      if (!merged.empty() &&
          frame.source_timestamp_ns - merged.back().source_timestamp_ns >
              150000000ULL) {
        catalog.annotations.push_back(
            {topic, "gap", frame.source_timestamp_ns,
             "相邻帧间隔超过150ms"});
      }
      merged.push_back(std::move(frame));
    }
    frames = std::move(merged);
  }
  return catalog;
}

LightweightMcapCatalog scan_mcap_directory_lightweight(
    const std::filesystem::path& directory,
    const std::vector<LidarChannelConfig>& channels) {
  std::unordered_map<std::string, LidarChannelConfig> wanted;
  for (const auto& channel : channels) {
    if (channel.enabled) wanted.emplace(channel.topic, channel);
  }
  LightweightMcapCatalog catalog;
  catalog.files = list_mcap_files(directory);
  std::uint64_t greatest_seen_bag_time{};
  std::vector<std::uint64_t> physical_bag_times;
  const FrameCallback summarize =
      [&](const std::string& topic, LidarFrame&& frame) {
        physical_bag_times.push_back(frame.bag_log_time_ns);
        if (frame.bag_log_time_ns < greatest_seen_bag_time) {
          catalog.max_observed_disorder_ns =
              std::max(catalog.max_observed_disorder_ns,
                       greatest_seen_bag_time - frame.bag_log_time_ns);
        } else {
          greatest_seen_bag_time = frame.bag_log_time_ns;
        }
        catalog.frames[topic].push_back(summarize_frame(frame));
      };
  for (const auto& path : catalog.files) {
    parse_file(path, wanted, summarize, &catalog.peak_chunk_bytes,
               kMaxStreamingChunkSize);
  }
  std::priority_queue<std::uint64_t, std::vector<std::uint64_t>,
                      std::greater<>>
      reorder;
  greatest_seen_bag_time = 0;
  for (const auto bag_time : physical_bag_times) {
    greatest_seen_bag_time =
        std::max(greatest_seen_bag_time, bag_time);
    reorder.push(bag_time);
    catalog.required_reorder_frames =
        std::max(catalog.required_reorder_frames, reorder.size());
    const auto watermark =
        greatest_seen_bag_time >= catalog.max_observed_disorder_ns
            ? greatest_seen_bag_time -
                  catalog.max_observed_disorder_ns
            : 0;
    while (!reorder.empty() && reorder.top() <= watermark) {
      reorder.pop();
    }
  }

  for (auto& [topic, frames] : catalog.frames) {
    const auto configured = wanted.find(topic);
    std::stable_sort(
        frames.begin(), frames.end(),
        [](const auto& left, const auto& right) {
          if (left.source_timestamp_ns !=
              right.source_timestamp_ns) {
            return left.source_timestamp_ns <
                   right.source_timestamp_ns;
          }
          return left.bag_log_time_ns < right.bag_log_time_ns;
        });
    std::vector<LidarFrameMetadata> merged;
    merged.reserve(frames.size());
    for (auto& frame : frames) {
      if (!merged.empty() &&
          merged.back().source_timestamp_ns == frame.source_timestamp_ns) {
        if (merged.back().payload_bytes != frame.payload_bytes ||
            merged.back().payload_sha256 != frame.payload_sha256) {
          throw std::runtime_error("同一topic同一时间戳内容冲突: " + topic);
        }
        ++catalog.duplicate_messages;
        continue;
      }
      merged.push_back(std::move(frame));
    }
    std::stable_sort(
        merged.begin(), merged.end(),
        [](const auto& left, const auto& right) {
          if (left.bag_log_time_ns != right.bag_log_time_ns) {
            return left.bag_log_time_ns < right.bag_log_time_ns;
          }
          return left.source_timestamp_ns <
                 right.source_timestamp_ns;
        });
    std::optional<LidarFrameMetadata> previous;
    const auto expected_protocol =
        configured->second.packet_size == 1118
            ? static_cast<std::uint8_t>(LidarPacketProtocol::MainUdp43)
            : static_cast<std::uint8_t>(LidarPacketProtocol::ShortUdp73);
    for (const auto& frame : merged) {
      if (frame.protocol != expected_protocol) {
        throw std::runtime_error("雷达topic协议与槽位配置不匹配: " +
                                 topic);
      }
      if (previous) {
        if (frame.first_packet_time_us <=
            previous->last_packet_time_us) {
          throw std::runtime_error("雷达包内时间未严格递增: " + topic);
        }
        const std::uint32_t sequence_delta =
            frame.first_packet_sequence -
            previous->last_packet_sequence;
        if (sequence_delta == 0 || sequence_delta >= 0x80000000U) {
          throw std::runtime_error(
              "雷达包内序号倒退或异常跳变: " + topic);
        }
        if (frame.bag_log_time_ns - previous->bag_log_time_ns >
            150000000ULL) {
          catalog.annotations.push_back(
              {topic, "gap", frame.bag_log_time_ns,
               "相邻帧间隔超过150ms"});
        }
      }
      previous = frame;
    }
    frames = std::move(merged);
  }
  return catalog;
}

McapStreamStats stream_mcap_frames(
    const LightweightMcapCatalog& catalog,
    const std::vector<LidarChannelConfig>& channels,
    McapStreamLimits limits, const McapFrameObserver& observer) {
  if (limits.max_reorder_frames == 0 || !observer) {
    throw std::invalid_argument("MCAP流式重排上限和observer必须有效");
  }
  if (catalog.required_reorder_frames > limits.max_reorder_frames) {
    throw std::runtime_error(
        "MCAP预检所需乱序帧缓冲超过明确上限: required=" +
        std::to_string(catalog.required_reorder_frames) +
        " limit=" + std::to_string(limits.max_reorder_frames));
  }
  std::unordered_map<std::string, LidarChannelConfig> wanted;
  std::unordered_map<std::string, std::size_t> channel_order;
  for (std::size_t index = 0; index < channels.size(); ++index) {
    if (channels[index].enabled) {
      wanted.emplace(channels[index].topic, channels[index]);
      channel_order.emplace(channels[index].topic, index);
    }
  }
  struct Pending {
    std::string topic;
    LidarFrame frame;
    std::size_t order{};
  };
  struct Later {
    bool operator()(const Pending& left, const Pending& right) const {
      if (left.frame.bag_log_time_ns != right.frame.bag_log_time_ns) {
        return left.frame.bag_log_time_ns > right.frame.bag_log_time_ns;
      }
      return left.order > right.order;
    }
  };
  std::priority_queue<Pending, std::vector<Pending>, Later> pending;
  McapStreamStats stats;
  std::uint64_t greatest_seen{};
  std::unordered_map<std::string,
                     std::unordered_set<std::uint64_t>>
      emitted_source_times;
  const auto emit_one = [&]() {
    auto item = std::move(const_cast<Pending&>(pending.top()));
    pending.pop();
    auto& emitted = emitted_source_times[item.topic];
    if (!emitted.insert(item.frame.source_timestamp_ns).second) {
      return true;
    }
    ++stats.frames_emitted;
    return observer(item.topic, std::move(item.frame));
  };
  bool keep_going = true;
  const FrameCallback collect =
      [&](const std::string& topic, LidarFrame&& frame) {
        if (!keep_going) return;
        greatest_seen = std::max(greatest_seen, frame.bag_log_time_ns);
        pending.push({topic, std::move(frame), channel_order.at(topic)});
        stats.max_reorder_frames =
            std::max(stats.max_reorder_frames, pending.size());
        if (pending.size() > limits.max_reorder_frames) {
          throw std::runtime_error(
              "MCAP流式重排缓冲超过明确上限: " +
              std::to_string(limits.max_reorder_frames));
        }
        const auto watermark =
            greatest_seen >= catalog.max_observed_disorder_ns
                ? greatest_seen - catalog.max_observed_disorder_ns
                : 0;
        while (!pending.empty() &&
               pending.top().frame.bag_log_time_ns <= watermark) {
          if (!emit_one()) {
            keep_going = false;
            break;
          }
        }
      };
  for (const auto& path : catalog.files) {
    if (!keep_going) break;
    parse_file(path, wanted, collect, nullptr,
               kMaxStreamingChunkSize);
  }
  while (keep_going && !pending.empty()) {
    keep_going = emit_one();
  }
  return stats;
}

}  // namespace joint_sensor_player
