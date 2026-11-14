// PTC 标定应答。域控连上来取 correction，和 UDP 点云不是同一条连接。

#include "joint_sensor_player/ptc_server.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

namespace joint_sensor_player {
namespace {

void send_all(int fd, const std::vector<std::uint8_t>& data) {
  std::size_t offset = 0;
  while (offset < data.size()) {
    const auto sent =
        ::send(fd, data.data() + offset, data.size() - offset, MSG_NOSIGNAL);
    if (sent <= 0) throw std::runtime_error("PTC响应发送失败");
    offset += static_cast<std::size_t>(sent);
  }
}

bool receive_exact(int fd, std::vector<std::uint8_t>& data) {
  std::size_t offset = 0;
  while (offset < data.size()) {
    const auto received =
        ::recv(fd, data.data() + offset, data.size() - offset, 0);
    if (received == 0) return false;
    if (received < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    offset += static_cast<std::size_t>(received);
  }
  return true;
}

}  // namespace

std::vector<std::uint8_t> build_ptc_response(
    const std::vector<std::uint8_t>& request,
    const std::vector<std::uint8_t>& correction) {
  if (request.size() != 8 || request[0] != 0x47 || request[1] != 0x74) {
    throw std::runtime_error("PTC 1.0请求头非法");
  }
  const std::uint32_t payload_size =
      (static_cast<std::uint32_t>(request[4]) << 24U) |
      (static_cast<std::uint32_t>(request[5]) << 16U) |
      (static_cast<std::uint32_t>(request[6]) << 8U) | request[7];
  if (payload_size != 0) {
    throw std::runtime_error("PTC标定请求不应携带负载");
  }

  const bool supported = request[2] == 0x05;
  const std::uint32_t response_size =
      supported ? static_cast<std::uint32_t>(correction.size()) : 0;
  std::vector<std::uint8_t> response;
  response.reserve(8 + response_size);
  response.push_back(0x47);
  response.push_back(0x74);
  response.push_back(request[2]);
  response.push_back(static_cast<std::uint8_t>(supported ? 0 : 1));
  response.push_back(
      static_cast<std::uint8_t>((response_size >> 24U) & 0xffU));
  response.push_back(
      static_cast<std::uint8_t>((response_size >> 16U) & 0xffU));
  response.push_back(
      static_cast<std::uint8_t>((response_size >> 8U) & 0xffU));
  response.push_back(static_cast<std::uint8_t>(response_size & 0xffU));
  if (supported) {
    for (const auto byte : correction) response.push_back(byte);
  }
  return response;
}

struct PtcCalibrationService::Impl {
  struct Listener {
    explicit Listener(PtcEndpoint value) : endpoint(std::move(value)) {}
    PtcEndpoint endpoint;
    int fd{-1};
    std::uint16_t bound_port{};
    std::thread thread;
  };

  explicit Impl(std::vector<PtcEndpoint> endpoints) {
    listeners.reserve(endpoints.size());
    for (auto& endpoint : endpoints) {
      listeners.push_back(std::make_unique<Listener>(std::move(endpoint)));
    }
  }

  ~Impl() { stop(); }

  void start() {
    if (running.exchange(true)) return;
    try {
      for (auto& item : listeners) {
        auto& listener = *item;
        listener.fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (listener.fd < 0) throw std::runtime_error("创建PTC socket失败");
        int reuse = 1;
        (void)::setsockopt(listener.fd, SOL_SOCKET, SO_REUSEADDR, &reuse,
                           sizeof(reuse));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(listener.endpoint.port);
        if (::inet_pton(AF_INET, listener.endpoint.address.c_str(),
                        &address.sin_addr) != 1) {
          throw std::runtime_error("PTC监听IP非法: " +
                                   listener.endpoint.address);
        }
        if (::bind(listener.fd, reinterpret_cast<sockaddr*>(&address),
                   sizeof(address)) != 0) {
          throw std::runtime_error("PTC绑定失败: " + listener.endpoint.address +
                                   ':' +
                                   std::to_string(listener.endpoint.port) +
                                   " (" + std::strerror(errno) + ')');
        }
        if (::listen(listener.fd, 4) != 0) {
          throw std::runtime_error("PTC监听失败");
        }
        socklen_t address_size = sizeof(address);
        if (::getsockname(listener.fd, reinterpret_cast<sockaddr*>(&address),
                          &address_size) != 0) {
          throw std::runtime_error("读取PTC监听端口失败");
        }
        listener.bound_port = ntohs(address.sin_port);
        listener.thread = std::thread([this, &listener] { run(listener); });
      }
    } catch (...) {
      stop();
      throw;
    }
  }

  void run(Listener& listener) {
    while (running.load()) {
      const int client = ::accept(listener.fd, nullptr, nullptr);
      if (client < 0) {
        if (!running.load()) return;
        if (errno == EINTR) continue;
        continue;
      }
      timeval timeout{0, 500000};
      (void)::setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                         sizeof(timeout));
      while (running.load()) {
        std::vector<std::uint8_t> request(8);
        if (!receive_exact(client, request)) break;
        try {
          send_all(client,
                   build_ptc_response(request, listener.endpoint.correction));
        } catch (...) {
          break;
        }
      }
      ::close(client);
    }
  }

  void stop() {
    running.store(false);
    for (auto& item : listeners) {
      if (item->fd >= 0) {
        ::shutdown(item->fd, SHUT_RDWR);
        ::close(item->fd);
        item->fd = -1;
      }
    }
    for (auto& item : listeners) {
      if (item->thread.joinable()) item->thread.join();
    }
  }

  std::atomic<bool> running{false};
  std::vector<std::unique_ptr<Listener>> listeners;
};

PtcCalibrationService::PtcCalibrationService(
    std::vector<PtcEndpoint> endpoints)
    : impl_(std::make_unique<Impl>(std::move(endpoints))) {}

PtcCalibrationService::~PtcCalibrationService() = default;

void PtcCalibrationService::start() { impl_->start(); }

std::vector<std::uint16_t> PtcCalibrationService::bound_ports() const {
  std::vector<std::uint16_t> ports;
  ports.reserve(impl_->listeners.size());
  for (const auto& listener : impl_->listeners) {
    ports.push_back(listener->bound_port);
  }
  return ports;
}

}  // namespace joint_sensor_player
