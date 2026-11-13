#include "joint_sensor_orchestrator/process.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <fcntl.h>

#include <array>
#include <cerrno>
#include <stdexcept>
#include <utility>

namespace joint_sensor_orchestrator {
namespace {

std::vector<char*> make_argv(const std::vector<std::string>& arguments) {
  if (arguments.empty() || arguments.front().empty()) {
    throw std::invalid_argument("子进程参数数组不能为空");
  }
  std::vector<char*> argv;
  argv.reserve(arguments.size() + 1);
  for (const auto& argument : arguments) {
    argv.push_back(const_cast<char*>(argument.c_str()));
  }
  argv.push_back(nullptr);
  return argv;
}

int wait_exit_code(pid_t child) {
  int status{};
  pid_t waited;
  do {
    waited = ::waitpid(child, &status, 0);
  } while (waited < 0 && errno == EINTR);
  if (waited < 0) throw std::runtime_error("waitpid等待子进程失败");
  if (WIFEXITED(status)) return WEXITSTATUS(status);
  if (WIFSIGNALED(status)) return 128 + WTERMSIG(status);
  return 126;
}

}  // namespace

int run_process_argv(const std::vector<std::string>& arguments) {
  auto argv = make_argv(arguments);
  const pid_t child = ::fork();
  if (child < 0) throw std::runtime_error("fork启动子进程失败");
  if (child == 0) {
    ::execv(argv.front(), argv.data());
    _exit(127);
  }
  return wait_exit_code(child);
}

ProcessResult run_process_argv_capture_stdout(
    const std::vector<std::string>& arguments) {
  auto argv = make_argv(arguments);
  int output_pipe[2]{};
  if (::pipe2(output_pipe, O_CLOEXEC) != 0) {
    throw std::runtime_error("创建子进程stdout管道失败");
  }
  const pid_t child = ::fork();
  if (child < 0) {
    ::close(output_pipe[0]);
    ::close(output_pipe[1]);
    throw std::runtime_error("fork启动子进程失败");
  }
  if (child == 0) {
    (void)::close(output_pipe[0]);
    if (::dup2(output_pipe[1], STDOUT_FILENO) < 0) _exit(127);
    (void)::close(output_pipe[1]);
    ::execv(argv.front(), argv.data());
    _exit(127);
  }
  (void)::close(output_pipe[1]);
  std::string output;
  std::array<char, 4096> buffer{};
  while (true) {
    const auto count = ::read(output_pipe[0], buffer.data(), buffer.size());
    if (count < 0 && errno == EINTR) continue;
    if (count < 0) {
      (void)::close(output_pipe[0]);
      (void)wait_exit_code(child);
      throw std::runtime_error("读取子进程stdout失败");
    }
    if (count == 0) break;
    output.append(buffer.data(), static_cast<std::size_t>(count));
  }
  (void)::close(output_pipe[0]);
  return {wait_exit_code(child), std::move(output)};
}

}  // namespace joint_sensor_orchestrator
