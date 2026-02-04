#pragma once

#include <chrono>
#include <mutex>

namespace agv_slam
{

struct RateTracker {
  std::chrono::steady_clock::time_point last_time;
  uint64_t count = 0;
  double hz = 0.0;
  std::mutex mtx;

  void tick() {
    std::lock_guard<std::mutex> lock(mtx);
    auto now = std::chrono::steady_clock::now();
    count++;
    if (count > 1) {
      double dt = std::chrono::duration<double>(now - last_time).count();
      hz = 0.9 * hz + 0.1 * (1.0 / dt);  // Exponential moving average
    }
    last_time = now;
  }

  double get_hz() {
    std::lock_guard<std::mutex> lock(mtx);
    return hz;
  }
};

}  // namespace agv_slam
