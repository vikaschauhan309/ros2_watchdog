#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "ros2_watchdog/action_dispatcher.hpp"
#include "ros2_watchdog/topic_monitor.hpp"

namespace ros2_watchdog
{

using LifecycleCallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Generic, config-driven health watchdog. Reads a list of topics to monitor
// from ROS parameters (see config/*.yaml for the schema), tracks each one
// with a TopicMonitor, and on every state transition asks the
// ActionDispatcher to run whatever actions are configured for that topic.
//
// Subscribes via rclcpp::GenericSubscription so it never needs the monitored
// topics' message types at compile time, and never deserializes payloads —
// only arrival timestamps matter for rate/staleness tracking.
class WatchdogNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit WatchdogNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  struct MonitoredTopic
  {
    TopicMonitorConfig monitor_config;
    std::vector<ActionConfig> actions;
    // Defaults to best-effort/volatile: a best-effort *subscriber* is QoS-
    // compatible with both best-effort and reliable publishers (DDS allows a
    // subscriber to request weaker reliability than what's offered), so this
    // is the one default that won't silently fail to connect to a sensor
    // driver publishing SensorDataQoS. Override per-topic if a publisher
    // genuinely requires reliable/transient_local to match.
    rclcpp::QoS qos = rclcpp::QoS(10).best_effort().durability_volatile();
    std::unique_ptr<TopicMonitor> monitor;
    rclcpp::GenericSubscription::SharedPtr subscription;
  };

  void LoadParameters();
  void ScanTimerCallback();
  void TeardownSubscriptionsAndTimer();

  std::vector<MonitoredTopic> monitored_topics_;
  std::unique_ptr<ActionDispatcher> dispatcher_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  double scan_period_sec_ = 0.1;
};

}  // namespace ros2_watchdog
