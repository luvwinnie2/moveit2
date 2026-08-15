# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""ザイルベアの解いた形状を RViz に出す可視化ノードの起動。

MoveIt が実際に衝突判定に使っている変形形状をそのまま描くので、
「計画は通ったがザイルベアが干渉していた」といった食い違いを目視で潰せる。

    ros2 launch moveit_cable_carrier carrier_visualizer.launch.py

既に move_group が動いている環境に足す想定。robot_description は
crx5ia_moveit の URDF から読む。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def _read(path):
    with open(path, "r") as handle:
        return handle.read()


def generate_launch_description():
    pkg = get_package_share_directory("moveit_cable_carrier")
    default_config = os.path.join(pkg, "config", "crx5ia_silveyer.yaml")

    try:
        moveit_pkg = get_package_share_directory("crx5ia_moveit")
        default_urdf = os.path.join(moveit_pkg, "urdf", "crx5ia.urdf")
        default_srdf = os.path.join(moveit_pkg, "config", "crx5ia.srdf")
    except Exception:
        default_urdf = ""
        default_srdf = ""

    args = [
        DeclareLaunchArgument("carrier_config", default_value=default_config),
        DeclareLaunchArgument("urdf", default_value=default_urdf),
        DeclareLaunchArgument("srdf", default_value=default_srdf),
        DeclareLaunchArgument("rate", default_value="20.0"),
    ]

    def _spawn(context):
        from launch_ros.actions import Node

        urdf_path = LaunchConfiguration("urdf").perform(context)
        srdf_path = LaunchConfiguration("srdf").perform(context)
        if not urdf_path or not os.path.exists(urdf_path):
            raise RuntimeError(f"URDF not found: '{urdf_path}' -- pass urdf:=<path>")

        return [
            Node(
                package="moveit_cable_carrier",
                executable="carrier_visualizer",
                name="carrier_visualizer",
                output="screen",
                parameters=[
                    {
                        "robot_description": _read(urdf_path),
                        "robot_description_semantic": _read(srdf_path) if srdf_path and os.path.exists(srdf_path) else "",
                        "carrier_config": LaunchConfiguration("carrier_config").perform(context),
                        "rate": float(LaunchConfiguration("rate").perform(context)),
                    }
                ],
            )
        ]

    from launch.actions import OpaqueFunction

    return LaunchDescription(args + [OpaqueFunction(function=_spawn)])
