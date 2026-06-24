"""
Integration test for the watchdog's stale/dead/recovery detection.

Launches the real watchdog node monitoring a topic the test itself
publishes to, then stops publishing and asserts a HealthEvent with
STATUS_STALE arrives on /watchdog/events within the configured window.
"""
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from rclpy.node import Node
from std_msgs.msg import Empty

from ros2_watchdog.msg import HealthEvent

TEST_TOPIC = "/test/watchdog_lifecycle_topic"
STALENESS_TIMEOUT_SEC = 0.3
DEAD_TIMEOUT_MULTIPLIER = 3.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    watchdog_node = launch_ros.actions.LifecycleNode(
        package="ros2_watchdog",
        executable="watchdog_node",
        name="ros2_watchdog",
        namespace="",
        output="screen",
        parameters=[
            {
                "scan_period_sec": 0.05,
                "monitored_topics": ["test_topic"],
                "monitored_topics.test_topic.topic": TEST_TOPIC,
                "monitored_topics.test_topic.type": "std_msgs/msg/Empty",
                "monitored_topics.test_topic.expected_rate_hz": 0.0,
                "monitored_topics.test_topic.staleness_timeout_sec": STALENESS_TIMEOUT_SEC,
                "monitored_topics.test_topic.dead_timeout_multiplier": DEAD_TIMEOUT_MULTIPLIER,
                "monitored_topics.test_topic.actions": ["publish_event"],
                "monitored_topics.test_topic.notify_on_recovery": True,
            }
        ],
    )

    configure_on_start = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(watchdog_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_on_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=watchdog_node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(watchdog_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return launch.LaunchDescription(
        [
            watchdog_node,
            activate_on_configured,
            configure_on_start,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestWatchdogDetectsStaleTopic(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node("test_watchdog_lifecycle_driver")
        self.publisher = self.node.create_publisher(Empty, TEST_TOPIC, 10)
        self.events = []
        self.node.create_subscription(
            HealthEvent, "/watchdog/events", self.events.append, 10
        )

    def tearDown(self):
        self.node.destroy_node()

    def _spin_for(self, seconds: float):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_topic_goes_stale_and_recovers(self):
        # Give the watchdog time to configure/activate and subscribe.
        self._spin_for(1.0)

        # Publish briefly so the watchdog sees the topic as alive first.
        for _ in range(5):
            self.publisher.publish(Empty())
            self._spin_for(0.1)

        # Now go silent past the staleness timeout and wait for the event.
        self._spin_for(STALENESS_TIMEOUT_SEC + 0.5)

        stale_events = [e for e in self.events if e.status == HealthEvent.STATUS_STALE]
        self.assertTrue(
            len(stale_events) > 0,
            f"expected a STALE HealthEvent, got statuses={[e.status for e in self.events]}",
        )

        # Publish again and confirm recovery is reported.
        for _ in range(5):
            self.publisher.publish(Empty())
            self._spin_for(0.1)

        ok_events = [e for e in self.events if e.status == HealthEvent.STATUS_OK]
        statuses = [e.status for e in self.events]
        self.assertTrue(
            len(ok_events) > 0,
            f"expected a recovery (OK) HealthEvent, got statuses={statuses}",
        )
