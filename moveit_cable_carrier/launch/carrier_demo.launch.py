# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""ザイルベア（3D ケーブルキャリア）の変形を目視するスタンドアロンデモ。

joint_state_publisher_gui のスライダで J4/J5/J6 を動かすと、MoveIt が衝突判定に
使っているのと同一の形状がリアルタイムで再計算され、RViz に描かれる。
形状が成立する姿勢は緑、成立しない姿勢（キャリアが届かない／曲げ半径を割る）は赤。

    ros2 launch moveit_cable_carrier carrier_demo.launch.py

【重要】Isaac Sim など /joint_states を出すものと同時に動かさないこと。
joint_state_publisher_gui はスライダの値を publish するので、両方動いていると
互いに上書きし合う。必ず片方だけにする。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _read(path):
    with open(path, "r") as handle:
        return handle.read()


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("moveit_cable_carrier")

    urdf_path = LaunchConfiguration("urdf").perform(context)
    carrier_config = LaunchConfiguration("carrier_config").perform(context)
    use_gui = LaunchConfiguration("gui").perform(context).lower() in ("true", "1", "yes")

    if not os.path.exists(urdf_path):
        raise RuntimeError(
            f"URDF not found: {urdf_path}\n"
            "生成するには:\n"
            "  ros2 run moveit_cable_carrier make_carrier_urdf.py "
            "<base.urdf> <carrier.yaml> <out.urdf>"
        )
    if not os.path.exists(carrier_config):
        raise RuntimeError(f"carrier config not found: {carrier_config}")

    robot_description = _read(urdf_path)

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="moveit_cable_carrier",
            executable="carrier_visualizer",
            name="carrier_visualizer",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "carrier_config": carrier_config,
                    "rate": 30.0,
                }
            ],
        ),
    ]

    if use_gui:
        nodes.append(
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            )
        )
    else:
        nodes.append(
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
            )
        )

    rviz_config = os.path.join(pkg, "config", "carrier_demo.rviz")
    if LaunchConfiguration("rviz").perform(context).lower() in ("true", "1", "yes"):
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            )
        )
    return nodes


def generate_launch_description():
    try:
        moveit_pkg = get_package_share_directory("crx5ia_moveit")
        default_urdf = os.path.join(moveit_pkg, "urdf", "crx5ia_with_carrier.urdf")
    except Exception:
        default_urdf = ""

    pkg = get_package_share_directory("moveit_cable_carrier")
    default_config = os.path.join(pkg, "config", "crx5ia_carrier.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("urdf", default_value=default_urdf),
            DeclareLaunchArgument("carrier_config", default_value=default_config),
            DeclareLaunchArgument("gui", default_value="true",
                                  description="joint_state_publisher_gui のスライダを出す"),
            DeclareLaunchArgument("rviz", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
