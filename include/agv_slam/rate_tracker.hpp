#pragma once

#include <chrono>
#include <mutex>
#include <cstdint>

namespace agv_slam
{

/// Tracks message rates with exponential moving average and frame drop detection.
struct RateTracker
{
  std::chrono::steady_clock::time_point last_time;
  uint64_t count = 0;
  double hz = 0.0;
  double expected_hz = 30.0;
  uint64_t drops = 0;
  std::mutex mtx;

  RateTracker() = default;

  explicit RateTracker(double expected)
  : expected_hz(expected) {}

  void tick()
  {
    std::lock_guard<std::mutex> lock(mtx);
    auto now = std::chrono::steady_clock::now();
    count++;
    if (count > 1) {
      double dt = std::chrono::duration<double>(now - last_time).count();
      hz = 0.9 * hz + 0.1 * (1.0 / dt);
      // Frame drop detection: if dt > 1.5x expected period, count as drop
      if (expected_hz > 0.0) {
        double expected_dt = 1.0 / expected_hz;
        if (dt > 1.5 * expected_dt) {
          uint64_t missed = static_cast<uint64_t>(dt / expected_dt) - 1;
          drops += (missed > 0) ? missed : 1;
        }
      }
    }
    last_time = now;
  }

  double get_hz()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return hz;
  }

  uint64_t get_count()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return count;
  }

  uint64_t get_drops()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return drops;
  }

  double drop_percent()
  {
    std::lock_guard<std::mutex> lock(mtx);
    if (count == 0) return 0.0;
    uint64_t total_expected = count + drops;
    return (total_expected > 0) ? (100.0 * drops / total_expected) : 0.0;
  }
};

}  // namespace agv_slam
