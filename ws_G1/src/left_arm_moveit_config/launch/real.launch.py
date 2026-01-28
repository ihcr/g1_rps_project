#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 与你的 SRDF/包名一致
    moveit_config = MoveItConfigsBuilder(
        "g1_dual_arm",                      # 机器人名
        package_name="left_arm_moveit_config"  # MoveIt 配置包
    ).to_moveit_configs()

    # 1) robot_state_publisher：用与 move_group 同源的 robot_description
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # 2) move_group（默认订阅 /joint_states）
    move_group = generate_move_group_launch(moveit_config)

    # 3) RViz（若该 rviz 配置不存在，可删除 arguments 行）
    rviz_cfg = PathJoinSubstitution([
        FindPackageShare("left_arm_moveit_config"), "config", "moveit.rviz"
    ])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
    )

    return LaunchDescription([rsp, move_group, rviz])
