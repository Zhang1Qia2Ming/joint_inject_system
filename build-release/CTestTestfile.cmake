# CMake generated Testfile for 
# Source directory: /home/ubuntu/zhang/joint_sensor_player
# Build directory: /home/ubuntu/zhang/joint_sensor_player/build-release
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(joint_sensor_player_tests "/home/ubuntu/zhang/joint_sensor_player/build-release/joint_sensor_player_tests")
set_tests_properties(joint_sensor_player_tests PROPERTIES  _BACKTRACE_TRIPLES "/home/ubuntu/zhang/joint_sensor_player/CMakeLists.txt;127;add_test;/home/ubuntu/zhang/joint_sensor_player/CMakeLists.txt;0;")
add_test(joint_sensor_orchestrator_tests "/home/ubuntu/zhang/joint_sensor_player/build-release/joint_sensor_orchestrator_tests")
set_tests_properties(joint_sensor_orchestrator_tests PROPERTIES  _BACKTRACE_TRIPLES "/home/ubuntu/zhang/joint_sensor_player/CMakeLists.txt;142;add_test;/home/ubuntu/zhang/joint_sensor_player/CMakeLists.txt;0;")
add_test(cli_help "/home/ubuntu/zhang/joint_sensor_player/build-release/joint_sensor_player" "--help")
set_tests_properties(cli_help PROPERTIES  _BACKTRACE_TRIPLES "/home/ubuntu/zhang/joint_sensor_player/CMakeLists.txt;146;add_test;/home/ubuntu/zhang/joint_sensor_player/CMakeLists.txt;0;")
add_test(emulator_cli_help "/home/ubuntu/zhang/joint_sensor_player/build-release/hil_lidar_emulator" "--help")
set_tests_properties(emulator_cli_help PROPERTIES  _BACKTRACE_TRIPLES "/home/ubuntu/zhang/joint_sensor_player/CMakeLists.txt;147;add_test;/home/ubuntu/zhang/joint_sensor_player/CMakeLists.txt;0;")
add_test(orchestrator_cli_help "/home/ubuntu/zhang/joint_sensor_player/build-release/joint_sensor_orchestrator" "--help")
set_tests_properties(orchestrator_cli_help PROPERTIES  _BACKTRACE_TRIPLES "/home/ubuntu/zhang/joint_sensor_player/CMakeLists.txt;148;add_test;/home/ubuntu/zhang/joint_sensor_player/CMakeLists.txt;0;")
add_test(video_cache_python "/usr/bin/python3.10" "-m" "unittest" "tests/test_build_video_cache.py" "-v")
set_tests_properties(video_cache_python PROPERTIES  WORKING_DIRECTORY "/home/ubuntu/zhang/joint_sensor_player" _BACKTRACE_TRIPLES "/home/ubuntu/zhang/joint_sensor_player/CMakeLists.txt;149;add_test;/home/ubuntu/zhang/joint_sensor_player/CMakeLists.txt;0;")
