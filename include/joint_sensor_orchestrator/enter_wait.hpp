#pragma once

// 开播前清掉已经堆在标准输入里的字节，再等一次真正的换行。
// 倒计时期间敲下的回车不能算作确认，否则会跳过预填。

#include <cerrno>

#include <fcntl.h>
#include <sys/select.h>
#include <unistd.h>

namespace joint_sensor_orchestrator {

enum class EnterWaitResult {
  Confirmed,
  Stopped,
  Closed,
};

// 丢掉输入缓冲里已经有的字节。倒计时期间敲下的换行不能当成后面的确认。
inline void discard_pending_input(int fd) {
  const int flags = ::fcntl(fd, F_GETFL);
  if (flags < 0) return;
  if (::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return;
  char buffer[256];
  while (true) {
    const ssize_t n = ::read(fd, buffer, sizeof(buffer));
    if (n < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (n == 0) break;
  }
  ::fcntl(fd, F_SETFL, flags);
}

// 清缓冲之后等到一个换行。select 或 read 出错不算确认。
template <typename Stop>
EnterWaitResult wait_for_newline(int fd, Stop&& stop) {
  while (!stop()) {
    fd_set reads;
    FD_ZERO(&reads);
    FD_SET(fd, &reads);
    timeval timeout{};
    timeout.tv_usec = 200000;
    const int ready = ::select(fd + 1, &reads, nullptr, nullptr, &timeout);
    if (ready < 0) {
      if (errno == EINTR) continue;
      return EnterWaitResult::Closed;
    }
    if (ready == 0) continue;
    char buffer[256];
    const ssize_t n = ::read(fd, buffer, sizeof(buffer));
    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      return EnterWaitResult::Closed;
    }
    if (n == 0) return EnterWaitResult::Closed;
    for (ssize_t index = 0; index < n; ++index) {
      if (buffer[index] == '\n') return EnterWaitResult::Confirmed;
    }
  }
  return EnterWaitResult::Stopped;
}

}  // namespace joint_sensor_orchestrator
