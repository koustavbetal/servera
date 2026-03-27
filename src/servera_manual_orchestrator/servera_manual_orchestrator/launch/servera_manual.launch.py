"""
launch/auroclean.launch.py

Starts:
  1. micro-ROS agent  (serial /dev/ttyACM0)
  2. auroclean_orchestrator node
  3. ros2 bag record   (all /auroclean/* topics)

Ctrl+C on this launch file kills all three cleanly.
The PySide UI is a separate process — do not include it here.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch_ros.actions import Node


PARAMS = os.path.join(
    os.path.dirname(__file__), "..", "config", "params.yaml"
)
BAG_DIR = os.path.expanduser("~/auroclean_bags")


def generate_launch_description():
    # ── micro-ROS agent ───────────────────────────────────────────
    agent = ExecuteProcess(
        cmd=[
            "ros2", "run", "micro_ros_agent", "micro_ros_agent",
            "serial", "--dev", "/dev/ttyACM0", "-b", "115200",
        ],
        name="micro_ros_agent",
        output="screen",
    )

    # ── Orchestrator ──────────────────────────────────────────────
    orchestrator = Node(
        package="auroclean_orchestrator",
        executable="orchestrator_node",
        name="auroclean_orchestrator",
        parameters=[PARAMS],
        output="screen",
        emulate_tty=True,
    )

    # ── ros2 bag — record all auroclean topics ────────────────────
    # Starts 2s after launch to let nodes come up first
    bag = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "record",
                    "-o", os.path.join(BAG_DIR,
                        "auroclean_{date}".replace("{date}", "")),
                    "/auroclean/cleaning_cmd",
                    "/auroclean/cleaning_status",
                    "/auroclean/motor_states",
                    "/auroclean/estop",
                ],
                name="rosbag",
                output="screen",
            )
        ],
    )

    # ── Log on orchestrator exit ───────────────────────────────────
    on_orch_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=orchestrator,
            on_exit=[LogInfo(msg="Orchestrator exited — session complete")],
        )
    )

    on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[LogInfo(msg="AuroClean launch shutting down")]
        )
    )

    return LaunchDescription([
        agent,
        orchestrator,
        # bag,
        on_orch_exit,
        on_shutdown,
    ])

