#include <robot_control/utils/timer.h>
#include <iostream>
#include <iomanip>

namespace rc::utils {

Timer::Timer() : clocks() {}

void Timer::start(std::string tag) {
  clocks[tag] = chrono::system_clock::now();
}

void Timer::stop(std::string tag) {
  TimePoint& start = clocks[tag];
  TimePoint now = chrono::high_resolution_clock::now();
  chrono::duration<double> diff = now - start;
  chrono::microseconds mu = chrono::duration_cast<chrono::microseconds>(diff);
  double ms = double(mu.count()) / 1000.0;
  std::cout << "Timing for " << tag << ": " << ms <<  "ms" << std::endl;
}

}
