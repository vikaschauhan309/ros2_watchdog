#include "ros2_watchdog/action_dispatcher.hpp"

#include <algorithm>

namespace ros2_watchdog
{

namespace
{
const char * StatusToString(HealthStatus status)
{
  switch (status) {
    case HealthStatus::kOk:
      return "OK";
    case HealthStatus::kStale:
      return "STALE";
    case HealthStatus::kDead:
      return "DEAD";
  }
  return "UNKNOWN";
}

uint8_t StatusToDiagnosticLevel(HealthStatus status)
{
  switch (status) {
    case HealthStatus::kOk:
      return diagnostic_msgs::msg::DiagnosticStatus::OK;
    case HealthStatus::kStale:
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    case HealthStatus::kDead:
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  return diagnostic_msgs::msg::DiagnosticStatus::STALE;
}

uint8_t StatusToHealthEventStatus(HealthStatus status)
{
  switch (status) {
    case HealthStatus::kOk:
      return ros2_watchdog::msg::HealthEvent::STATUS_OK;
    case HealthStatus::kStale:
      return ros2_watchdog::msg::HealthEvent::STATUS_STALE;
    case HealthStatus::kDead:
      return ros2_watchdog::msg::HealthEvent::STATUS_DEAD;
  }
  return ros2_watchdog::msg::HealthEvent::STATUS_DEAD;
}
}  // namespace

ActionDispatcher::ActionDispatcher(rclcpp_lifecycle::LifecycleNode * node)
: node_(node), logger_(node->get_logger())
{
  diagnostic_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/watchdog/diagnostics", rclcpp::QoS(10));
  health_event_pub_ = node_->create_publisher<ros2_watchdog::msg::HealthEvent>(
    "/watchdog/events", rclcpp::QoS(10));
}

void ActionDispatcher::Activate()
{
  diagnostic_pub_->on_activate();
  health_event_pub_->on_activate();
}

void ActionDispatcher::Deactivate()
{
  diagnostic_pub_->on_deactivate();
  health_event_pub_->on_deactivate();
}

void ActionDispatcher::RegisterTopicActions(
  const std::string & topic_name, std::vector<ActionConfig> actions)
{
  topic_actions_[topic_name] = std::move(actions);
}

void ActionDispatcher::Dispatch(
  const std::string & topic_name, HealthStatus status, double estimated_rate_hz)
{
  auto it = topic_actions_.find(topic_name);
  if (it == topic_actions_.end()) {
    return;
  }
  for (const auto & action : it->second) {
    const bool should_fire =
      std::find(action.trigger_on.begin(), action.trigger_on.end(), status) !=
      action.trigger_on.end();
    if (should_fire) {
      RunAction(action, topic_name, status, estimated_rate_hz);
    }
  }
}

void ActionDispatcher::RunAction(
  const ActionConfig & action, const std::string & topic_name, HealthStatus status,
  double estimated_rate_hz)
{
  switch (action.type) {
    case ActionType::kLog: {
        if (status == HealthStatus::kOk) {
          RCLCPP_INFO(
            logger_, "[%s] recovered (rate=%.2f Hz)", topic_name.c_str(), estimated_rate_hz);
        } else {
          RCLCPP_WARN(
            logger_, "[%s] is %s (rate=%.2f Hz)", topic_name.c_str(), StatusToString(status),
            estimated_rate_hz);
        }
        break;
      }
    case ActionType::kPublishDiagnostic: {
        diagnostic_msgs::msg::DiagnosticStatus msg;
        msg.name = topic_name;
        msg.level = StatusToDiagnosticLevel(status);
        msg.message = std::string("ros2_watchdog: ") + StatusToString(status);
        diagnostic_pub_->publish(msg);
        break;
      }
    case ActionType::kPublishEvent: {
        ros2_watchdog::msg::HealthEvent msg;
        msg.topic_name = topic_name;
        msg.status = StatusToHealthEventStatus(status);
        msg.message = StatusToString(status);
        msg.stamp = node_->get_clock()->now();
        health_event_pub_->publish(msg);
        break;
      }
    case ActionType::kCallTriggerService: {
        auto client = GetOrCreateTriggerClient(action.service_name);
        if (!client->service_is_ready()) {
          RCLCPP_WARN(
            logger_, "[%s] action target service '%s' is not available, skipping call",
            topic_name.c_str(), action.service_name.c_str());
          return;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        const std::string service_name = action.service_name;
        using TriggerFuture = rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture;
        client->async_send_request(
          request,
          [this, topic_name, service_name](TriggerFuture future) {
            const auto response = future.get();
            RCLCPP_INFO(
              logger_, "[%s] called '%s': success=%d message='%s'", topic_name.c_str(),
              service_name.c_str(), response->success, response->message.c_str());
          });
        break;
      }
  }
}

rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ActionDispatcher::GetOrCreateTriggerClient(
  const std::string & service_name)
{
  auto it = trigger_clients_.find(service_name);
  if (it != trigger_clients_.end()) {
    return it->second;
  }
  auto client = node_->create_client<std_srvs::srv::Trigger>(service_name);
  trigger_clients_[service_name] = client;
  return client;
}

}  // namespace ros2_watchdog
