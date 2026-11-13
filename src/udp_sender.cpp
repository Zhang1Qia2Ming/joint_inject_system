#include "joint_sensor_player/udp_sender.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <array>
#include <cstring>
#include <stdexcept>

namespace joint_sensor_player {

struct UdpDatagramSink::Target {
  int fd{-1};
  unsigned interface_index{};
  in_addr source_address{};
  sockaddr_in destination{};
};

UdpDatagramSink::UdpDatagramSink(const PlayerConfig& config) {
  try {
    for (const auto& channel : config.channels) {
      if (!channel.enabled) continue;
      Target target;
      target.interface_index = ::if_nametoindex(config.interface.c_str());
      if (target.interface_index == 0) {
        throw std::runtime_error("无法解析网口索引: " + config.interface);
      }
      target.fd = ::socket(AF_INET, SOCK_DGRAM, 0);
      if (target.fd < 0) throw std::runtime_error("创建UDP socket失败");
      const int send_buffer = 4 * 1024 * 1024;
      if (::setsockopt(target.fd, SOL_SOCKET, SO_SNDBUF, &send_buffer,
                       sizeof(send_buffer)) != 0) {
        const std::string detail = std::strerror(errno);
        ::close(target.fd);
        throw std::runtime_error("设置UDP发送缓冲失败: " + detail);
      }
      const int flags = ::fcntl(target.fd, F_GETFL, 0);
      if (flags < 0 ||
          ::fcntl(target.fd, F_SETFL, flags | O_NONBLOCK) != 0) {
        const std::string detail = std::strerror(errno);
        ::close(target.fd);
        throw std::runtime_error("设置UDP非阻塞失败: " + detail);
      }

      sockaddr_in source{};
      source.sin_family = AF_INET;
      source.sin_port = htons(channel.source_port);
      if (::inet_pton(AF_INET, channel.source_ip.c_str(), &source.sin_addr) !=
          1) {
        ::close(target.fd);
        throw std::runtime_error("源IP格式非法: " + channel.source_ip);
      }
      target.source_address = source.sin_addr;
      if (::bind(target.fd, reinterpret_cast<sockaddr*>(&source),
                 sizeof(source)) != 0) {
        const std::string detail = std::strerror(errno);
        ::close(target.fd);
        throw std::runtime_error("绑定" + channel.source_ip + ":" +
                                 std::to_string(channel.source_port) +
                                 "失败: " + detail);
      }
      target.destination.sin_family = AF_INET;
      target.destination.sin_port = htons(channel.destination_port);
      if (::inet_pton(AF_INET, config.destination_ip.c_str(),
                      &target.destination.sin_addr) != 1) {
        ::close(target.fd);
        throw std::runtime_error("目标IP格式非法");
      }
      targets_.emplace(channel.slot, target);
    }
  } catch (...) {
    for (auto& [slot, target] : targets_) {
      (void)slot;
      if (target.fd >= 0) ::close(target.fd);
    }
    throw;
  }
}

UdpDatagramSink::~UdpDatagramSink() {
  for (auto& [slot, target] : targets_) {
    (void)slot;
    if (target.fd >= 0) ::close(target.fd);
  }
}

bool UdpDatagramSink::send(const LidarChannelConfig& channel,
                           const std::vector<std::uint8_t>& payload,
                           std::uint64_t deadline_ns, std::string& error) {
  (void)deadline_ns;
  const auto found = targets_.find(channel.slot);
  if (found == targets_.end()) {
    error = "没有slot " + std::to_string(channel.slot) + " 的socket";
    return false;
  }
  iovec vector{};
  vector.iov_base = const_cast<std::uint8_t*>(payload.data());
  vector.iov_len = payload.size();
  std::array<char, CMSG_SPACE(sizeof(in_pktinfo))> control{};
  msghdr message{};
  message.msg_name = &found->second.destination;
  message.msg_namelen = sizeof(found->second.destination);
  message.msg_iov = &vector;
  message.msg_iovlen = 1;
  message.msg_control = control.data();
  message.msg_controllen = control.size();
  auto* cmsg = CMSG_FIRSTHDR(&message);
  cmsg->cmsg_level = IPPROTO_IP;
  cmsg->cmsg_type = IP_PKTINFO;
  cmsg->cmsg_len = CMSG_LEN(sizeof(in_pktinfo));
  auto* packet_info = reinterpret_cast<in_pktinfo*>(CMSG_DATA(cmsg));
  packet_info->ipi_ifindex =
      static_cast<int>(found->second.interface_index);
  packet_info->ipi_spec_dst = found->second.source_address;
  const auto sent = ::sendmsg(found->second.fd, &message, MSG_DONTWAIT);
  if (sent < 0 || static_cast<std::size_t>(sent) != payload.size()) {
    error = "slot " + std::to_string(channel.slot) +
            " sendto失败: " + std::strerror(errno);
    return false;
  }
  return true;
}

}  // namespace joint_sensor_player
