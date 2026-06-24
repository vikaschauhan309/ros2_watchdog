#include <gtest/gtest.h>

#include <chrono>

#include "ros2_watchdog/topic_monitor.hpp"

using ros2_watchdog::HealthStatus;
using ros2_watchdog::TopicMonitor;
using ros2_watchdog::TopicMonitorConfig;
using Clock = std::chrono::steady_clock;

namespace
{
TopicMonitorConfig MakeConfig()
{
  TopicMonitorConfig config;
  config.topic_name = "/test/topic";
  config.topic_type = "std_msgs/msg/Empty";
  config.expected_rate_hz = 10.0;
  config.rate_tolerance_ratio = 0.5;
  config.staleness_timeout_sec = 1.0;
  config.dead_timeout_multiplier = 5.0;
  config.rate_window_size = 10;
  return config;
}
}  // namespace

TEST(TopicMonitor, StartsOk)
{
  auto now = Clock::now();
  TopicMonitor monitor(MakeConfig(), now);
  EXPECT_EQ(monitor.status(), HealthStatus::kOk);
}

TEST(TopicMonitor, GoesStaleAfterTimeoutWithNoMessages)
{
  auto now = Clock::now();
  TopicMonitor monitor(MakeConfig(), now);

  monitor.Update(now + std::chrono::milliseconds(500));
  EXPECT_EQ(monitor.status(), HealthStatus::kOk);

  monitor.Update(now + std::chrono::milliseconds(1100));
  EXPECT_EQ(monitor.status(), HealthStatus::kStale);
}

TEST(TopicMonitor, GoesDeadAfterDeadTimeout)
{
  auto now = Clock::now();
  TopicMonitor monitor(MakeConfig(), now);

  monitor.Update(now + std::chrono::milliseconds(5100));
  EXPECT_EQ(monitor.status(), HealthStatus::kDead);
}

TEST(TopicMonitor, MessageArrivalRecoversToOk)
{
  auto now = Clock::now();
  TopicMonitor monitor(MakeConfig(), now);

  monitor.Update(now + std::chrono::milliseconds(1100));
  EXPECT_EQ(monitor.status(), HealthStatus::kStale);

  auto recovered_time = now + std::chrono::milliseconds(1200);
  monitor.OnMessageReceived(recovered_time);
  EXPECT_EQ(monitor.status(), HealthStatus::kOk);

  monitor.Update(recovered_time + std::chrono::milliseconds(100));
  EXPECT_EQ(monitor.status(), HealthStatus::kOk);
}

TEST(TopicMonitor, UpdateReturnsTrueOnlyOnTransition)
{
  auto now = Clock::now();
  TopicMonitor monitor(MakeConfig(), now);

  EXPECT_FALSE(monitor.Update(now + std::chrono::milliseconds(100)));
  EXPECT_TRUE(monitor.Update(now + std::chrono::milliseconds(1100)));
  EXPECT_FALSE(monitor.Update(now + std::chrono::milliseconds(1200)));
}

TEST(TopicMonitor, DegradedRateTriggersStaleEvenWithRecentMessages)
{
  auto now = Clock::now();
  TopicMonitor monitor(MakeConfig(), now);

  // Feed messages at 4 Hz (below the 5 Hz floor: 10 Hz expected * 0.5 tolerance)
  // — frequent enough to never hit the staleness timeout, but slow enough to
  // violate the rate tolerance.
  auto t = now;
  for (int i = 0; i < 5; ++i) {
    t += std::chrono::milliseconds(250);
    monitor.OnMessageReceived(t);
    monitor.Update(t);
  }

  EXPECT_EQ(monitor.status(), HealthStatus::kStale);
}

TEST(TopicMonitor, RateEstimateResetsAfterOutageRatherThanFlapping)
{
  auto now = Clock::now();
  TopicMonitor monitor(MakeConfig(), now);

  // Establish a healthy 10 Hz baseline.
  auto t = now;
  for (int i = 0; i < 5; ++i) {
    t += std::chrono::milliseconds(100);
    monitor.OnMessageReceived(t);
  }

  // Go silent past the dead timeout, then resume at the same healthy rate.
  // Without resetting the rate window, the first post-outage sample would
  // pair with a pre-outage timestamp and report a falsely low Hz estimate.
  t += std::chrono::milliseconds(5100);
  monitor.OnMessageReceived(t);
  monitor.Update(t);
  EXPECT_EQ(monitor.status(), HealthStatus::kOk);

  t += std::chrono::milliseconds(100);
  monitor.OnMessageReceived(t);
  EXPECT_EQ(monitor.status(), HealthStatus::kOk);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
