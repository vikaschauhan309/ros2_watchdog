#include "ros2_watchdog/watchdog_node.hpp"

#include <chrono>
#include <stdexcept>

namespace ros2_watchdog
{

namespace
{
ActionType ParseActionType(const std::string & name)
{
  if (name == "log") {
    return ActionType::kLog;
  }
  if (name == "publish_diagnostic") {
    return ActionType::kPublishDiagnostic;
  }
  if (name == "publish_event") {
    return ActionType::kPublishEvent;
  }
  if (name == "call_trigger_service") {
    return ActionType::kCallTriggerService;
  }
  throw std::invalid_argument("Unknown watchdog action type: '" + name + "'");
}

rclcpp::QoS ParseQos(
  const std::string & reliability, const std::string & durability, int depth,
  const std::string & topic_key, rclcpp::Logger logger)
{
  rclcpp::QoS qos(static_cast<size_t>(std::max(depth, 1)));

  if (reliability == "best_effort") {
    qos.best_effort();
  } else if (reliability == "reliable") {
    qos.reliable();
  } else {
    RCLCPP_WARN(
      logger, "Topic key '%s' has unknown qos_reliability '%s', defaulting to best_effort",
      topic_key.c_str(), reliability.c_str());
    qos.best_effort();
  }

  if (durability == "volatile") {
    qos.durability_volatile();
  } else if (durability == "transient_local") {
    qos.transient_local();
  } else {
    RCLCPP_WARN(
      logger, "Topic key '%s' has unknown qos_durability '%s', defaulting to volatile",
      topic_key.c_str(), durability.c_str());
    qos.durability_volatile();
  }

  return qos;
}
}  // namespace

WatchdogNode::WatchdogNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("ros2_watchdog", options)
{
}

void WatchdogNode::LoadParameters()
{
  monitored_topics_.clear();

  scan_period_sec_ = declare_parameter<double>("scan_period_sec", 0.1);
  const auto topic_keys = declare_parameter<std::vector<std::string>>(
    "monitored_topics", std::vector<std::string>{});

  for (const auto & key : topic_keys) {
    const std::string prefix = "monitored_topics." + key + ".";

    TopicMonitorConfig monitor_config;
    monitor_config.topic_name = declare_parameter<std::string>(prefix + "topic", "");
    monitor_config.topic_type = declare_parameter<std::string>(prefix + "type", "");
    monitor_config.expected_rate_hz = declare_parameter<double>(prefix + "expected_rate_hz", 0.0);
    monitor_config.rate_tolerance_ratio =
      declare_parameter<double>(prefix + "rate_tolerance_ratio", 0.5);
    monitor_config.staleness_timeout_sec =
      declare_parameter<double>(prefix + "staleness_timeout_sec", 1.0);
    monitor_config.dead_timeout_multiplier =
      declare_parameter<double>(prefix + "dead_timeout_multiplier", 5.0);

    if (monitor_config.topic_name.empty() || monitor_config.topic_type.empty()) {
      RCLCPP_WARN(
        get_logger(), "Skipping monitored topic '%s': 'topic' and 'type' are required",
        key.c_str());
      continue;
    }

    const auto action_names = declare_parameter<std::vector<std::string>>(
      prefix + "actions", std::vector<std::string>{"log"});
    const std::string trigger_service = declare_parameter<std::string>(
      prefix + "trigger_service", "");
    const bool notify_on_recovery = declare_parameter<bool>(prefix + "notify_on_recovery", true);

    std::vector<ActionConfig> actions;
    for (const auto & action_name : action_names) {
      ActionConfig action;
      try {
        action.type = ParseActionType(action_name);
      } catch (const std::invalid_argument & e) {
        RCLCPP_WARN(get_logger(), "%s (topic key '%s')", e.what(), key.c_str());
        continue;
      }
      action.trigger_on = {HealthStatus::kStale, HealthStatus::kDead};
      if (action.type == ActionType::kCallTriggerService) {
        if (trigger_service.empty()) {
          RCLCPP_WARN(
            get_logger(),
            "Topic key '%s' has a call_trigger_service action but no trigger_service set, skipping",
            key.c_str());
          continue;
        }
        action.service_name = trigger_service;
      } else if (notify_on_recovery) {
        action.trigger_on.push_back(HealthStatus::kOk);
      }
      actions.push_back(action);
    }

    const std::string qos_reliability =
      declare_parameter<std::string>(prefix + "qos_reliability", "best_effort");
    const std::string qos_durability =
      declare_parameter<std::string>(prefix + "qos_durability", "volatile");
    const int qos_depth = declare_parameter<int>(prefix + "qos_depth", 10);

    MonitoredTopic entry;
    entry.monitor_config = monitor_config;
    entry.actions = std::move(actions);
    entry.qos = ParseQos(qos_reliability, qos_durability, qos_depth, key, get_logger());
    monitored_topics_.push_back(std::move(entry));
  }
}

LifecycleCallbackReturn WatchdogNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    LoadParameters();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load watchdog parameters: %s", e.what());
    return LifecycleCallbackReturn::FAILURE;
  }

  dispatcher_ = std::make_unique<ActionDispatcher>(this);

  const auto now = std::chrono::steady_clock::now();
  for (auto & entry : monitored_topics_) {
    entry.monitor = std::make_unique<TopicMonitor>(entry.monitor_config, now);
    dispatcher_->RegisterTopicActions(entry.monitor_config.topic_name, entry.actions);

    entry.subscription = rclcpp::create_generic_subscription(
      get_node_topics_interface(), entry.monitor_config.topic_name,
      entry.monitor_config.topic_type, entry.qos,
      [this, topic_name = entry.monitor_config.topic_name](
        std::shared_ptr<rclcpp::SerializedMessage>) {
        for (auto & m : monitored_topics_) {
          if (m.monitor_config.topic_name == topic_name) {
            if (m.monitor->OnMessageReceived(std::chrono::steady_clock::now())) {
              dispatcher_->Dispatch(
                topic_name, m.monitor->status(), m.monitor->estimated_rate_hz());
            }
            break;
          }
        }
      });

    RCLCPP_INFO(
      get_logger(), "Monitoring '%s' (type=%s, expected_rate=%.2f Hz, staleness_timeout=%.2f s)",
      entry.monitor_config.topic_name.c_str(), entry.monitor_config.topic_type.c_str(),
      entry.monitor_config.expected_rate_hz, entry.monitor_config.staleness_timeout_sec);
  }

  RCLCPP_INFO(get_logger(), "Configured with %zu monitored topics", monitored_topics_.size());
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn WatchdogNode::on_activate(const rclcpp_lifecycle::State &)
{
  dispatcher_->Activate();
  scan_timer_ = create_wall_timer(
    std::chrono::duration<double>(scan_period_sec_), [this]() {ScanTimerCallback();});
  RCLCPP_INFO(get_logger(), "ros2_watchdog activated, scanning every %.3f s", scan_period_sec_);
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn WatchdogNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  scan_timer_.reset();
  dispatcher_->Deactivate();
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn WatchdogNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  TeardownSubscriptionsAndTimer();
  monitored_topics_.clear();
  dispatcher_.reset();
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn WatchdogNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  TeardownSubscriptionsAndTimer();
  return LifecycleCallbackReturn::SUCCESS;
}

void WatchdogNode::TeardownSubscriptionsAndTimer()
{
  scan_timer_.reset();
  for (auto & entry : monitored_topics_) {
    entry.subscription.reset();
  }
}

void WatchdogNode::ScanTimerCallback()
{
  const auto now = std::chrono::steady_clock::now();
  for (auto & entry : monitored_topics_) {
    if (entry.monitor->Update(now)) {
      dispatcher_->Dispatch(
        entry.monitor_config.topic_name, entry.monitor->status(),
        entry.monitor->estimated_rate_hz());
    }
  }
}

}  // namespace ros2_watchdog
