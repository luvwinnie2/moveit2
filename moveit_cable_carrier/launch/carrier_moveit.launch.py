# Copyright 2026 Leow Chee Siang. Apache-2.0.
"""MoveIt 2 一式 + ザイルベア衝突判定 + RViz2（MotionPlanning パネル付き）。

    ros2 launch moveit_cable_carrier carrier_moveit.launch.py

これで立ち上がるもの:
  robot_state_publisher    ブラケット座標系入りの URDF を配信
  joint_state_publisher_gui スライダで姿勢を作る（オフラインデモ）
  move_group               **collision_detector: CABLE_CARRIER** で起動するので、
                           MotionPlanning パネルからの計画がザイルベアを衝突物として扱う
  carrier_visualizer       解いた形状をマーカーで出す。パネルから実行時調整できる
  rviz2                    MotionPlanning パネル + Cable carrier パネル

【重要】Isaac Sim など /joint_states を出すものと同時に動かさないこと。
joint_state_publisher_gui と競合して関節値を奪い合う。
Isaac と併用する場合は gui:=false で起動する。

ザイルベアが衝突判定に入っているかの確認方法:
    ros2 param get /move_group collision_detector
        -> moveit_cable_carrier/CABLE_CARRIER
move_group の起動ログにも "CollisionEnvCarrier: N cable carrier(s) configured" が出る。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("moveit_cable_carrier")
    carrier_config = LaunchConfiguration("carrier_config").perform(context)
    urdf_rel = LaunchConfiguration("urdf").perform(context)
    use_carrier = LaunchConfiguration("use_carrier_collision").perform(context).lower() in ("true", "1", "yes")

    if not os.path.exists(carrier_config):
        raise RuntimeError(f"carrier config not found: {carrier_config}")

    moveit_config = (
        MoveItConfigsBuilder("crx5ia", package_name="crx5ia_moveit")
        .robot_description(file_path=urdf_rel)
        .robot_description_semantic(file_path="config/crx5ia.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    move_group_params = [moveit_config.to_dict(), {"use_sim_time": False}]
    if use_carrier:
        # CollisionPluginLoader::setupScene reads this parameter off the move_group node and
        # activates the matching pluginlib class, so every planning query -- including the diff
        # scenes move_group makes per request -- goes through the carrier-aware environment.
        move_group_params.append({"collision_detector": "moveit_cable_carrier/CABLE_CARRIER"})

    # The collision detector is constructed by MoveIt's allocator template, which passes only a
    # robot model and a world; there is no route for per-detector configuration. The registry
    # therefore reads this environment variable on first use.
    carrier_env = {"MOVEIT_CABLE_CARRIER_CONFIG": carrier_config}

    robot_description_text = moveit_config.robot_description["robot_description"]
    robot_semantic_text = moveit_config.robot_description_semantic["robot_description_semantic"]

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[moveit_config.robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        # Deliberately no joint_state_publisher when gui:=false. carrier_visualizer publishes
        # /joint_states itself in drive_robot mode, and two publishers would fight: the slider set
        # would keep resetting whatever the marker just solved for.
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("static_joint_source")),
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=move_group_params,
            additional_env=carrier_env,
        ),
        Node(
            package="moveit_cable_carrier",
            executable="carrier_visualizer",
            name="carrier_visualizer",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description_text,
                    "robot_description_semantic": robot_semantic_text,
                    "carrier_config": carrier_config,
                    "planning_group": "arm",
                    "rate": 20.0,
                    "drive_robot": False,
                },
                # Needed for the IK that makes the carrier follow the end-effector marker while it
                # is dragged. Without it the node logs that the group has no solver and simply
                # stops following, rather than failing.
                moveit_config.robot_description_kinematics,
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", os.path.join(pkg, "config", "carrier_moveit.rviz")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ),
    ]
    return nodes


def generate_launch_description():
    pkg = get_package_share_directory("moveit_cable_carrier")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "carrier_config",
                default_value=os.path.join(pkg, "config", "crx5ia_carrier.yaml"),
            ),
            DeclareLaunchArgument(
                "urdf",
                default_value="urdf/crx5ia_with_carrier.urdf",
                description="crx5ia_moveit からの相対パス。ブラケット座標系入りの版を既定にしている",
            ),
            DeclareLaunchArgument("use_carrier_collision", default_value="true"),
            DeclareLaunchArgument(
                "gui", default_value="false",
                description="関節スライダを出す。static_joint_source と排他"),
            DeclareLaunchArgument(
                "static_joint_source", default_value="true",
                description="joint_state_publisher を使う。マーカー追従（drive_robot）は未完成のため既定はこちら"),
            DeclareLaunchArgument("rviz", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
