#include <gtest/gtest.h>
#include "agv_slam/rate_tracker.hpp"
#include <cmath>
#include <thread>
#include <chrono>
#include <vector>

using agv_slam::RateTracker;

TEST(RateTracker, InitialHzIsZero)
{
  RateTracker tracker;
  EXPECT_DOUBLE_EQ(tracker.get_hz(), 0.0);
  EXPECT_EQ(tracker.count, 0u);
}

TEST(RateTracker, SingleTickNoRate)
{
  RateTracker tracker;
  tracker.tick();
  // After one tick, hz should still be 0 — need at least two for a rate
  EXPECT_DOUBLE_EQ(tracker.get_hz(), 0.0);
  EXPECT_EQ(tracker.count, 1u);
}

TEST(RateTracker, DualTickProducesRate)
{
  RateTracker tracker;
  tracker.tick();
  // Sleep ~50ms to get a measurable dt
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  tracker.tick();

  double hz = tracker.get_hz();
  // EMA with α=0.1: first sample yields 0.1 * (1/dt). With 50ms → 0.1*20 ≈ 2Hz.
  EXPECT_GT(hz, 0.5);
  EXPECT_LT(hz, 100.0);
  EXPECT_EQ(tracker.count, 2u);
}

TEST(RateTracker, EmaConvergesToSteadyRate)
{
  RateTracker tracker;
  // Simulate ~100Hz stream: 20 ticks at 10ms intervals
  for (int i = 0; i < 20; i++) {
    tracker.tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  double hz = tracker.get_hz();
  // Should converge toward 100Hz. Allow generous range due to scheduling.
  EXPECT_GT(hz, 30.0);
  EXPECT_LT(hz, 200.0);
}

TEST(RateTracker, ThreadSafetyConcurrentTicks)
{
  RateTracker tracker;

  auto tick_fn = [&tracker]() {
    for (int i = 0; i < 100; i++) {
      tracker.tick();
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  };

  std::vector<std::thread> threads;
  for (int t = 0; t < 4; t++) {
    threads.emplace_back(tick_fn);
  }
  for (auto & t : threads) {
    t.join();
  }

  // All ticks should be counted (4 threads * 100 ticks)
  EXPECT_EQ(tracker.count, 400u);
  // Should have a valid rate (not NaN or negative)
  double hz = tracker.get_hz();
  EXPECT_GE(hz, 0.0);
  EXPECT_FALSE(std::isnan(hz));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
