#!/usr/bin/env python3
"""
Publishes /demo/sensor at 10 Hz, then deliberately goes silent.

Demonstrates the watchdog's OK -> STALE -> DEAD -> action -> RECOVERED
cycle (matches config/demo_params.yaml).

Run alongside the watchdog launch, then watch:
    ros2 topic echo /watchdog/events
"""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class DemoFailureSim(Node):
    def __init__(self, silent_seconds: float):
        super().__init__("demo_failure_sim")
        self._pub = self.create_publisher(Empty, "/demo/sensor", 10)
        self._silent_seconds = silent_seconds
        self._timer = self.create_timer(0.1, self._tick)
        self._start_time = self.get_clock().now()
        self._announced_silence = False
        self._announced_resume = False

    def _tick(self):
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        in_silent_window = 3.0 <= elapsed < 3.0 + self._silent_seconds

        if in_silent_window:
            if not self._announced_silence:
                self.get_logger().warn(
                    f"going silent on /demo/sensor for {self._silent_seconds:.1f}s..."
                )
                self._announced_silence = True
            return

        if self._announced_silence and not self._announced_resume:
            self.get_logger().info("resuming /demo/sensor publishing")
            self._announced_resume = True

        self._pub.publish(Empty())


def main():
    silent_seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    rclpy.init()
    node = DemoFailureSim(silent_seconds)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
