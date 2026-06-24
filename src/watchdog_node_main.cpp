#include <rclcpp/rclcpp.hpp>

#include "ros2_watchdog/watchdog_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_watchdog::WatchdogNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
