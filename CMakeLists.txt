cmake_minimum_required(VERSION 3.16)
project(joint_sensor_player VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

add_library(joint_sensor_player_lib
  src/clock_health.cpp
  src/config.cpp
  src/mcap.cpp
  src/packet_sequence.cpp
  src/packet_time_rewriter.cpp
  src/ptc_server.cpp
  src/timeline.cpp
  src/report.cpp
  src/udp_sender.cpp
  src/player.cpp
)
target_include_directories(joint_sensor_player_lib PUBLIC include)
target_link_libraries(joint_sensor_player_lib
  PUBLIC
    Threads::Threads
    yaml-cpp
    nlohmann_json::nlohmann_json
    OpenSSL::Crypto
    PkgConfig::ZSTD
)
target_compile_options(joint_sensor_player_lib PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_library(joint_sensor_orchestrator_lib
  src/orchestrator_config.cpp
  src/ptp_schedule.cpp
  src/lifecycle_manager.cpp
  src/video_cache.cpp
  src/unified_timeline.cpp
  src/video_frame_queue.cpp
  src/frame_buffer_pool.cpp
  src/mock_video_sink.cpp
  src/xdma_video_sink.cpp
  src/start_gate.cpp
  src/radar_engine.cpp
  src/video_engine.cpp
  src/report_v2.cpp
  src/orchestrator.cpp
  src/process.cpp
  src/dataset_validation.cpp
  src/cache_preparer.cpp
  src/smoke_radar.cpp
  src/video_prefault.cpp
  src/video_ahead_cache.cpp
  src/file_pin.cpp
  src/thread_affinity.cpp
  src/shadow_video.cpp
)
target_include_directories(joint_sensor_orchestrator_lib PUBLIC include)
target_link_libraries(joint_sensor_orchestrator_lib
  PUBLIC
    joint_sensor_player_lib
    OpenSSL::Crypto
)
target_compile_options(joint_sensor_orchestrator_lib PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

add_executable(joint_sensor_player src/main.cpp)
target_link_libraries(joint_sensor_player PRIVATE joint_sensor_player_lib)
target_compile_options(joint_sensor_player PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(hil_lidar_emulator src/emulator_main.cpp)
target_link_libraries(hil_lidar_emulator PRIVATE joint_sensor_player_lib)
target_compile_options(hil_lidar_emulator PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(joint_sensor_orchestrator src/orchestrator_main.cpp)
target_link_libraries(joint_sensor_orchestrator PRIVATE
  joint_sensor_orchestrator_lib
)
target_compile_definitions(joint_sensor_orchestrator PRIVATE
  JOINT_SENSOR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_compile_options(joint_sensor_orchestrator PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)

include(CTest)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(joint_sensor_player_tests
    tests/test_main.cpp
    tests/test_clock_health.cpp
    tests/test_config.cpp
    tests/test_mcap.cpp
    tests/test_packet_sequence.cpp
    tests/test_packet_time_rewriter.cpp
    tests/test_ptc_server.cpp
    tests/test_timeline.cpp
    tests/test_report.cpp
    tests/test_player.cpp
    tests/test_orchestrator_config.cpp
    tests/test_lifecycle_manager.cpp
    tests/test_video_cache.cpp
    tests/test_unified_timeline.cpp
    tests/test_video_frame_queue.cpp
    tests/test_video_sink.cpp
    tests/test_start_gate.cpp
    tests/test_radar_engine.cpp
    tests/test_video_engine.cpp
    tests/test_dataset_validation.cpp
    tests/test_smoke_radar.cpp
    tests/test_video_prefault.cpp
    tests/test_video_ahead_cache.cpp
    tests/test_file_pin.cpp
    tests/test_thread_affinity.cpp
    tests/test_shadow_video.cpp
    tests/test_ptp_schedule.cpp
  )
  target_link_libraries(joint_sensor_player_tests PRIVATE
    joint_sensor_player_lib
    joint_sensor_orchestrator_lib
  )
  target_compile_options(joint_sensor_player_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME joint_sensor_player_tests COMMAND joint_sensor_player_tests)
  add_executable(joint_sensor_orchestrator_tests
    tests/test_main.cpp
    tests/test_report_v2.cpp
    tests/test_orchestrator_integration.cpp
    tests/test_process.cpp
    tests/test_orchestrator_cli.cpp
  )
  target_link_libraries(joint_sensor_orchestrator_tests PRIVATE
    joint_sensor_orchestrator_lib
  )
  target_compile_options(joint_sensor_orchestrator_tests PRIVATE
    -Wall -Wextra -Wpedantic -Werror
  )
  add_dependencies(joint_sensor_orchestrator_tests joint_sensor_orchestrator)
  add_test(
    NAME joint_sensor_orchestrator_tests
    COMMAND joint_sensor_orchestrator_tests
  )
  add_test(NAME cli_help COMMAND joint_sensor_player --help)
  add_test(NAME emulator_cli_help COMMAND hil_lidar_emulator --help)
  add_test(NAME orchestrator_cli_help COMMAND joint_sensor_orchestrator --help)
  add_test(
    NAME video_cache_python
    COMMAND ${Python3_EXECUTABLE} -m unittest tests/test_build_video_cache.py -v
  )
  set_tests_properties(video_cache_python PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()
