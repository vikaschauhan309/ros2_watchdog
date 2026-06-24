#pragma once

#include <chrono>
#include <cstddef>
#include <deque>
#include <string>

namespace ros2_watchdog
{

enum class HealthStatus
{
  kOk,
  kStale,
  kDead,
};

struct TopicMonitorConfig
{
  std::string topic_name;
  std::string topic_type;
  double expected_rate_hz = 0.0;   // 0 disables rate checking, staleness-only
  double rate_tolerance_ratio = 0.5;  // allowed fractional deviation from expected_rate_hz
  double staleness_timeout_sec = 1.0;  // no message for this long => STALE
  double dead_timeout_multiplier = 5.0;  // STALE for staleness_timeout_sec * this => DEAD
  std::size_t rate_window_size = 10;  // number of recent arrivals used for Hz estimate
};

// Pure state machine: tracks message arrivals for a single topic and derives
// OK/STALE/DEAD health from wall-clock gaps. Has no ROS dependency so it can
// be unit tested without spinning a node.
class TopicMonitor
{
public:
  TopicMonitor(TopicMonitorConfig config, std::chrono::steady_clock::time_point now);

  // Call when a message arrives on the monitored topic. Returns true if the
  // status changed (e.g. STALE/DEAD -> OK) as a result.
  bool OnMessageReceived(std::chrono::steady_clock::time_point now);

  // Call periodically (faster than staleness_timeout_sec) to re-evaluate state
  // based on elapsed time since the last message. Returns true if the status
  // changed since the last time either this or OnMessageReceived reported one.
  bool Update(std::chrono::steady_clock::time_point now);

  HealthStatus status() const {return status_;}
  double estimated_rate_hz() const {return estimated_rate_hz_;}
  const std::string & topic_name() const {return config_.topic_name;}
  const TopicMonitorConfig & config() const {return config_;}

private:
  void RecomputeEstimatedRate();

  // Returns true and updates last_reported_status_ if status_ has diverged
  // from it. Single chokepoint so OnMessageReceived's immediate status flip
  // and Update()'s periodic re-evaluation report transitions consistently.
  bool ConsumeTransition();

  TopicMonitorConfig config_;
  HealthStatus status_ = HealthStatus::kOk;
  HealthStatus last_reported_status_ = HealthStatus::kOk;
  bool has_received_message_ = false;
  // Used as the staleness reference clock until the first message arrives,
  // so a topic that never publishes still ages into STALE/DEAD correctly.
  std::chrono::steady_clock::time_point reference_time_;
  std::deque<std::chrono::steady_clock::time_point> recent_arrivals_;
  double estimated_rate_hz_ = 0.0;
};

}  // namespace ros2_watchdog
