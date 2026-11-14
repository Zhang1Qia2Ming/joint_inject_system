#pragma once

// 拉起缓存构建这类外部程序，并把退出码和标准输出交回来。

#include <string>
#include <vector>

namespace joint_sensor_orchestrator {

struct ProcessResult {
  int exit_code{};
  std::string stdout_text;
};

int run_process_argv(const std::vector<std::string>& arguments);
ProcessResult run_process_argv_capture_stdout(
    const std::vector<std::string>& arguments);

}  // namespace joint_sensor_orchestrator
