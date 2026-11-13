#include "joint_sensor_orchestrator/process.hpp"
#include "test_framework.hpp"

#include <unistd.h>

#include <filesystem>
#include <fstream>

TEST_CASE(process_runner_uses_literal_argument_vector_without_shell) {
  const auto root = std::filesystem::temp_directory_path() /
                    ("process-argv-" + std::to_string(::getpid()));
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  const auto script = root / "capture.py";
  const auto output = root / "args.txt";
  const auto injected = root / "injected";
  {
    std::ofstream file(script);
    file << "import pathlib,sys\n"
            "pathlib.Path(sys.argv[1]).write_text(sys.argv[2])\n";
  }
  const std::string hostile = "literal;touch " + injected.string();
  const int status = joint_sensor_orchestrator::run_process_argv(
      {"/usr/bin/python3", script.string(), output.string(), hostile});

  CHECK(status == 0);
  std::ifstream input(output);
  std::string captured;
  std::getline(input, captured);
  CHECK(captured == hostile);
  CHECK(!std::filesystem::exists(injected));
  std::filesystem::remove_all(root);
}

TEST_CASE(process_runner_returns_child_failure_status) {
  CHECK(joint_sensor_orchestrator::run_process_argv(
            {"/usr/bin/python3", "-c", "raise SystemExit(7)"}) == 7);
  CHECK_THROWS(joint_sensor_orchestrator::run_process_argv({}));
}

TEST_CASE(process_runner_captures_literal_stdout_for_cache_key_parsing) {
  const auto result = joint_sensor_orchestrator::run_process_argv_capture_stdout(
      {"/usr/bin/python3", "-c", "print('/cache/abc123')"});
  CHECK(result.exit_code == 0);
  CHECK(result.stdout_text == "/cache/abc123\n");
}
