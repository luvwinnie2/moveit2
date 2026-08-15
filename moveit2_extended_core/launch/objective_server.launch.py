# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""Standalone objective server: core Behaviors only, no robot required.

    ros2 launch moveit2_extended_core objective_server.launch.py

Then, in another shell:

    ros2 service call /objective_server/list_behaviors \\
        moveit2_extended_msgs/srv/ListBehaviors
    ros2 action send_goal -f /objective_server/execute_objective \\
        moveit2_extended_msgs/action/ExecuteObjective \\
        "{objective_name: SmokeTest,
          parameters: [{name: caller, value: {type: 4, string_value: cli}}]}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("moveit2_extended_core")
    default_config = os.path.join(share, "config", "objective_server.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="objective server parameter file",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for the server",
            ),
            Node(
                package="moveit2_extended_core",
                executable="objective_server",
                name="objective_server",
                output="screen",
                parameters=[LaunchConfiguration("config")],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
        ]
    )
