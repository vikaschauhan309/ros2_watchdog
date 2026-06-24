"""
Launches the watchdog lifecycle node and auto-transitions it to ACTIVE.

Usage:
    ros2 launch ros2_watchdog watchdog.launch.py
    ros2 launch ros2_watchdog watchdog.launch.py params_file:=config/demo_params.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("ros2_watchdog"), "config", "watchdog_params.yaml"
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to the watchdog YAML parameter file",
    )

    watchdog_node = LifecycleNode(
        package="ros2_watchdog",
        executable="watchdog_node",
        name="ros2_watchdog",
        namespace="",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    # Drive the lifecycle node straight to ACTIVE on startup: configure once
    # it spawns, then activate once configure succeeds.
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

    return LaunchDescription(
        [
            params_file_arg,
            watchdog_node,
            activate_on_configured,
            configure_on_start,
        ]
    )
