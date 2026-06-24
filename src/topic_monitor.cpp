#include "ros2_watchdog/topic_monitor.hpp"

namespace ros2_watchdog
{

namespace
{
double SecondsBetween(
  std::chrono::steady_clock::time_point earlier,
  std::chrono::steady_clock::time_point later)
{
  return std::chrono::duration<double>(later - earlier).count();
}
}  // namespace

TopicMonitor::TopicMonitor(TopicMonitorConfig config, std::chrono::steady_clock::time_point now)
: config_(std::move(config)), reference_time_(now)
{
}

bool TopicMonitor::OnMessageReceived(std::chrono::steady_clock::time_point now)
{
  // A gap longer than the staleness timeout makes the existing window
  // unrepresentative of the current rate (it would span the outage), so
  // start the rate estimate fresh rather than report a skewed low Hz value
  // for the next rate_window_size messages.
  if (!recent_arrivals_.empty() &&
    SecondsBetween(recent_arrivals_.back(), now) >= config_.staleness_timeout_sec)
  {
    recent_arrivals_.clear();
  }

  has_received_message_ = true;
  reference_time_ = now;

  recent_arrivals_.push_back(now);
  while (recent_arrivals_.size() > config_.rate_window_size) {
    recent_arrivals_.pop_front();
  }
  RecomputeEstimatedRate();

  // Any fresh message immediately clears STALE/DEAD back to OK. Rate-based
  // degradation (if configured) is re-evaluated on the next Update() tick.
  status_ = HealthStatus::kOk;
  return ConsumeTransition();
}

bool TopicMonitor::Update(std::chrono::steady_clock::time_point now)
{
  const double elapsed_since_reference = SecondsBetween(reference_time_, now);
  const double dead_timeout_sec = config_.staleness_timeout_sec * config_.dead_timeout_multiplier;

  const bool rate_degraded = recent_arrivals_.size() >= 2 && config_.expected_rate_hz > 0.0 &&
    estimated_rate_hz_ < config_.expected_rate_hz * (1.0 - config_.rate_tolerance_ratio);

  if (elapsed_since_reference >= dead_timeout_sec) {
    status_ = HealthStatus::kDead;
  } else if (elapsed_since_reference >= config_.staleness_timeout_sec) {
    status_ = HealthStatus::kStale;
  } else if (rate_degraded) {
    status_ = HealthStatus::kStale;
  } else {
    status_ = HealthStatus::kOk;
  }

  return ConsumeTransition();
}

bool TopicMonitor::ConsumeTransition()
{
  if (status_ == last_reported_status_) {
    return false;
  }
  last_reported_status_ = status_;
  return true;
}

void TopicMonitor::RecomputeEstimatedRate()
{
  if (recent_arrivals_.size() < 2) {
    estimated_rate_hz_ = 0.0;
    return;
  }
  const double span_sec = SecondsBetween(recent_arrivals_.front(), recent_arrivals_.back());
  if (span_sec <= 0.0) {
    estimated_rate_hz_ = 0.0;
    return;
  }
  estimated_rate_hz_ = static_cast<double>(recent_arrivals_.size() - 1) / span_sec;
}

}  // namespace ros2_watchdog
