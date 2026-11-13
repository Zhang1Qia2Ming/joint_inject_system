#pragma once

#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace testfw {

using Test = std::pair<std::string, std::function<void()>>;

inline std::vector<Test>& registry() {
  static std::vector<Test> tests;
  return tests;
}

struct Register {
  Register(std::string name, std::function<void()> fn) {
    registry().emplace_back(std::move(name), std::move(fn));
  }
};

inline void fail(const char* expr, const char* file, int line) {
  std::ostringstream out;
  out << file << ':' << line << ": assertion failed: " << expr;
  throw std::runtime_error(out.str());
}

}  // namespace testfw

#define TEST_CASE(name)                                                        \
  static void test_##name();                                                   \
  static testfw::Register register_##name(#name, test_##name);                 \
  static void test_##name()

#define CHECK(expr)                                                            \
  do {                                                                         \
    if (!(expr)) testfw::fail(#expr, __FILE__, __LINE__);                      \
  } while (false)

#define CHECK_THROWS(expr)                                                     \
  do {                                                                         \
    bool threw = false;                                                        \
    try {                                                                      \
      (void)(expr);                                                            \
    } catch (...) {                                                            \
      threw = true;                                                            \
    }                                                                          \
    if (!threw) testfw::fail("expected exception: " #expr, __FILE__, __LINE__); \
  } while (false)
