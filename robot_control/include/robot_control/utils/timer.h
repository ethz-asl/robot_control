#pragma once
#include <map>
#include <chrono>

namespace chrono = std::chrono;
using TimePoint = chrono::high_resolution_clock::time_point;

namespace rc::utils {
class Timer {
  std::map<std::string, chrono::high_resolution_clock::time_point> clocks;
  public:
  Timer();
  void start(std::string tag);
  void stop(std::string tag);
};
}
