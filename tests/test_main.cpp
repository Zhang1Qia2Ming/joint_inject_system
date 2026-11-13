#include "test_framework.hpp"

#include <iostream>

int main() {
  int failed = 0;
  for (const auto& [name, test] : testfw::registry()) {
    try {
      test();
      std::cout << "[PASS] " << name << '\n';
    } catch (const std::exception& error) {
      ++failed;
      std::cerr << "[FAIL] " << name << ": " << error.what() << '\n';
    }
  }
  std::cout << testfw::registry().size() - failed << '/'
            << testfw::registry().size() << " tests passed\n";
  return failed == 0 ? 0 : 1;
}
