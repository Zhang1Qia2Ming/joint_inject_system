#pragma once

#include <zstd.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace testdata {

inline void append_u16(std::vector<std::uint8_t>& out, std::uint16_t value) {
  const auto* p = reinterpret_cast<const std::uint8_t*>(&value);
  out.insert(out.end(), p, p + sizeof(value));
}
inline void append_u32(std::vector<std::uint8_t>& out, std::uint32_t value) {
  const auto* p = reinterpret_cast<const std::uint8_t*>(&value);
  out.insert(out.end(), p, p + sizeof(value));
}
inline void append_i32(std::vector<std::uint8_t>& out, std::int32_t value) {
  const auto* p = reinterpret_cast<const std::uint8_t*>(&value);
  out.insert(out.end(), p, p + sizeof(value));
}
inline void append_u64(std::vector<std::uint8_t>& out, std::uint64_t value) {
  const auto* p = reinterpret_cast<const std::uint8_t*>(&value);
  out.insert(out.end(), p, p + sizeof(value));
}
inline void append_string(std::vector<std::uint8_t>& out, const std::string& value) {
  append_u32(out, static_cast<std::uint32_t>(value.size()));
  out.insert(out.end(), value.begin(), value.end());
}
inline void append_record(std::vector<std::uint8_t>& out, std::uint8_t opcode,
                          const std::vector<std::uint8_t>& body) {
  out.push_back(opcode);
  append_u64(out, body.size());
  out.insert(out.end(), body.begin(), body.end());
}

inline std::vector<std::uint8_t> channel(std::uint16_t id,
                                         const std::string& topic) {
  std::vector<std::uint8_t> body;
  append_u16(body, id);
  append_u16(body, 0);
  append_string(body, topic);
  append_string(body, "cdr");
  append_u32(body, 0);
  std::vector<std::uint8_t> record;
  append_record(record, 0x04, body);
  return record;
}

inline std::vector<std::uint8_t> udp_frame(std::int32_t sec, std::uint32_t nsec,
                                           std::size_t packet_size,
                                           std::uint32_t packet_count,
                                           std::uint8_t fill) {
  std::vector<std::uint8_t> data{0, 1, 0, 0};
  append_i32(data, sec);
  append_u32(data, nsec);
  const std::string frame = "lidar";
  data.insert(data.end(), frame.begin(), frame.end());
  data.resize(44, 0);
  append_u32(data, packet_count);
  for (std::uint32_t i = 0; i < packet_count; ++i) {
    append_i32(data, 0);
    append_u32(data, 0);
    data.insert(data.end(), packet_size, static_cast<std::uint8_t>(fill + i));
    while ((data.size() % 4) != 0) data.push_back(0);
    append_u32(data, static_cast<std::uint32_t>(packet_size));
  }
  return data;
}

inline std::vector<std::uint8_t> udp_frame_packets(
    std::int32_t sec, std::uint32_t nsec,
    const std::vector<std::vector<std::uint8_t>>& packets) {
  std::vector<std::uint8_t> data{0, 1, 0, 0};
  append_i32(data, sec);
  append_u32(data, nsec);
  const std::string frame = "lidar";
  data.insert(data.end(), frame.begin(), frame.end());
  data.resize(44, 0);
  append_u32(data, static_cast<std::uint32_t>(packets.size()));
  for (const auto& packet : packets) {
    append_i32(data, 0);
    append_u32(data, 0);
    data.insert(data.end(), packet.begin(), packet.end());
    while ((data.size() % 4) != 0) data.push_back(0);
    append_u32(data, static_cast<std::uint32_t>(packet.size()));
  }
  return data;
}

inline std::vector<std::uint8_t> main_lidar_packet(
    std::uint64_t time_us, std::uint32_t sequence) {
  std::vector<std::uint8_t> packet(1118);
  packet[0] = 0xEE;
  packet[1] = 0xFF;
  packet[2] = 0x04;
  packet[3] = 0x03;
  packet[6] = 128;
  packet[7] = 2;
  packet[11] = 1;
  const auto micros = static_cast<std::uint32_t>(time_us % 1'000'000ULL);
  std::memcpy(packet.data() + 1066, &micros, sizeof(micros));
  const auto seconds = static_cast<std::uint32_t>(time_us / 1'000'000ULL);
  packet[1074] = static_cast<std::uint8_t>(seconds >> 24U);
  packet[1075] = static_cast<std::uint8_t>(seconds >> 16U);
  packet[1076] = static_cast<std::uint8_t>(seconds >> 8U);
  packet[1077] = static_cast<std::uint8_t>(seconds);
  std::memcpy(packet.data() + 1078, &sequence, sizeof(sequence));
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t index = 1046; index < 1082; ++index) {
    crc ^= packet[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1U) ^
            (0xEDB88320U &
             (0U - static_cast<std::uint32_t>(crc & 1U)));
    }
  }
  crc ^= 0xFFFFFFFFU;
  std::memcpy(packet.data() + 1082, &crc, sizeof(crc));
  return packet;
}

inline std::vector<std::uint8_t> message(std::uint16_t channel_id,
                                         std::uint32_t sequence,
                                         std::uint64_t log_time,
                                         const std::vector<std::uint8_t>& data) {
  std::vector<std::uint8_t> body;
  append_u16(body, channel_id);
  append_u32(body, sequence);
  append_u64(body, log_time);
  append_u64(body, log_time);
  body.insert(body.end(), data.begin(), data.end());
  std::vector<std::uint8_t> record;
  append_record(record, 0x05, body);
  return record;
}

inline void write_mcap(const std::filesystem::path& path,
                       const std::string& topic,
                       const std::vector<std::vector<std::uint8_t>>& messages,
                       bool zstd_chunk) {
  std::vector<std::uint8_t> bytes{
      0x89, 'M', 'C', 'A', 'P', '0', '\r', '\n'};
  const auto channel_record = channel(1, topic);
  bytes.insert(bytes.end(), channel_record.begin(), channel_record.end());
  if (!zstd_chunk) {
    for (const auto& record : messages) {
      bytes.insert(bytes.end(), record.begin(), record.end());
    }
  } else {
    std::vector<std::uint8_t> raw;
    for (const auto& record : messages) raw.insert(raw.end(), record.begin(), record.end());
    std::vector<std::uint8_t> compressed(ZSTD_compressBound(raw.size()));
    const auto size = ZSTD_compress(compressed.data(), compressed.size(), raw.data(),
                                    raw.size(), 1);
    if (ZSTD_isError(size)) throw std::runtime_error("zstd test compression failed");
    compressed.resize(size);
    std::vector<std::uint8_t> chunk;
    append_u64(chunk, 0);
    append_u64(chunk, 0);
    append_u64(chunk, raw.size());
    append_u32(chunk, 0);
    append_string(chunk, "zstd");
    append_u64(chunk, compressed.size());
    chunk.insert(chunk.end(), compressed.begin(), compressed.end());
    append_record(bytes, 0x06, chunk);
  }
  std::vector<std::uint8_t> data_end;
  append_u32(data_end, 0);
  append_record(bytes, 0x0F, data_end);
  bytes.insert(bytes.end(), {0x89, 'M', 'C', 'A', 'P', '0', '\r', '\n'});
  std::ofstream out(path, std::ios::binary);
  out.write(reinterpret_cast<const char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
}

}  // namespace testdata
