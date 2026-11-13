#include "test_framework.hpp"

#include "joint_sensor_player/ptc_server.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <vector>

using joint_sensor_player::build_ptc_response;
using joint_sensor_player::PtcCalibrationService;
using joint_sensor_player::PtcEndpoint;

TEST_CASE(ptc_calibration_command_returns_binary_correction) {
  const std::vector<std::uint8_t> request{0x47, 0x74, 0x05, 0x00,
                                          0x00, 0x00, 0x00, 0x00};
  const std::vector<std::uint8_t> correction{0xee, 0xff, 0x03, 0x01};

  const auto response = build_ptc_response(request, correction);

  CHECK(response == std::vector<std::uint8_t>(
                        {0x47, 0x74, 0x05, 0x00, 0x00, 0x00, 0x00, 0x04,
                         0xee, 0xff, 0x03, 0x01}));
}

TEST_CASE(ptc_unsupported_command_returns_error_without_payload) {
  const std::vector<std::uint8_t> request{0x47, 0x74, 0x09, 0x00,
                                          0x00, 0x00, 0x00, 0x00};

  const auto response = build_ptc_response(request, {0x01});

  CHECK(response == std::vector<std::uint8_t>(
                        {0x47, 0x74, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00}));
}

TEST_CASE(ptc_rejects_invalid_or_nonempty_calibration_request) {
  CHECK_THROWS(build_ptc_response({0x00, 0x74, 0x05, 0x00,
                                   0x00, 0x00, 0x00, 0x00},
                                  {0x01}));
  CHECK_THROWS(build_ptc_response({0x47, 0x74, 0x05, 0x00,
                                   0x00, 0x00, 0x00, 0x01, 0x00},
                                  {0x01}));
}

TEST_CASE(ptc_service_serves_calibration_over_tcp) {
  PtcCalibrationService service(
      {PtcEndpoint{"127.0.0.1", 0, {0xee, 0xff, 0x03}}});
  service.start();
  const auto port = service.bound_ports().at(0);

  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  CHECK(fd >= 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  CHECK(::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) == 1);
  CHECK(::connect(fd, reinterpret_cast<sockaddr*>(&address),
                  sizeof(address)) == 0);
  const std::vector<std::uint8_t> request{0x47, 0x74, 0x05, 0x00,
                                          0x00, 0x00, 0x00, 0x00};
  CHECK(::send(fd, request.data(), request.size(), 0) ==
        static_cast<ssize_t>(request.size()));
  std::vector<std::uint8_t> response(11);
  std::size_t received = 0;
  while (received < response.size()) {
    const auto count =
        ::recv(fd, response.data() + received, response.size() - received, 0);
    CHECK(count > 0);
    received += static_cast<std::size_t>(count);
  }
  ::close(fd);
  CHECK(response == std::vector<std::uint8_t>(
                        {0x47, 0x74, 0x05, 0x00, 0x00, 0x00, 0x00, 0x03,
                         0xee, 0xff, 0x03}));
}
