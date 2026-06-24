#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros2_watchdog/msg/health_event.hpp"
#include "ros2_watchdog/topic_monitor.hpp"

namespace ros2_watchdog
{

enum class ActionType
{
  kLog,
  kPublishDiagnostic,
  kCallTriggerService,
  kPublishEvent,
};

struct ActionConfig
{
  ActionType type = ActionType::kLog;
  std::string service_name;  // only used for kCallTriggerService
  // Which health states this action fires on. Defaults to STALE and DEAD;
  // include kOk to also notify on recovery.
  std::vector<HealthStatus> trigger_on{HealthStatus::kStale, HealthStatus::kDead};
};

// Executes the configured action(s) for a topic on a health state transition.
// Deliberately a simple dispatch switch rather than a plugin framework: the
// four action types cover what a YAML config needs to express, and adding a
// fifth is a one-function change, not an extension point worth abstracting yet.
class ActionDispatcher
{
public:
  explicit ActionDispatcher(rclcpp_lifecycle::LifecycleNode * node);

  void RegisterTopicActions(const std::string & topic_name, std::vector<ActionConfig> actions);

  // Call on every health state transition reported by a TopicMonitor.
  void Dispatch(const std::string & topic_name, HealthStatus status, double estimated_rate_hz);

  // LifecyclePublishers must be explicitly activated/deactivated in step with
  // the owning node's lifecycle state, or publish() silently no-ops.
  void Activate();
  void Deactivate();

private:
  void RunAction(
    const ActionConfig & action, const std::string & topic_name, HealthStatus status,
    double estimated_rate_hz);
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr GetOrCreateTriggerClient(
    const std::string & service_name);

  rclcpp_lifecycle::LifecycleNode * node_;
  rclcpp::Logger logger_;
  std::unordered_map<std::string, std::vector<ActionConfig>> topic_actions_;
  std::unordered_map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr>
  trigger_clients_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
    diagnostic_pub_;
  rclcpp_lifecycle::LifecyclePublisher<ros2_watchdog::msg::HealthEvent>::SharedPtr
    health_event_pub_;
};

}  // namespace ros2_watchdog
