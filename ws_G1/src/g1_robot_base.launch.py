#!/usr/bin/env python3
"""
G1 机器人底层控制 Launch 文件

启动内容：
1. ros2 run left_moveit real_gripper_control_right (右手夹爪控制)
2. ros2 run g1arm_moveit g1arm_moveit (手臂控制节点)
3. bridge_dual_arm.py --hz 100 (双臂关节状态桥接)

使用方法：
    # 先设置环境变量
    source /opt/ros/humble/setup.bash
    source ~/unitree_ros2/setup.sh
    source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=0
    source ~/ws_G1/install/setup.bash

    # 启动
    ros2 launch ~/ws_G1/src/g1_robot_base.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    home_dir = os.path.expanduser('~')
    ws_g1_dir = os.path.join(home_dir, 'ws_G1')

    # ROS2 环境设置命令
    ros2_env = (
        'source /opt/ros/humble/setup.bash && '
        'source ~/unitree_ros2/setup.sh && '
        'source ~/unitree_ros2/cyclonedds_ws/install/setup.bash && '
        'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && '
        'export ROS_DOMAIN_ID=0 && '
        'source ~/ws_G1/install/setup.bash'
    )

    return LaunchDescription([
        # ============================================
        # 1. real_gripper_control_right (立即启动)
        #    右手夹爪控制，订阅 /rps/ai_move
        # ============================================
        ExecuteProcess(
            cmd=['bash', '-c',
                f'{ros2_env} && ros2 run left_moveit real_gripper_control_right'],
            cwd=ws_g1_dir,
            output='screen',
            name='real_gripper_control_right'
        ),

        # ============================================
        # 2. g1arm_moveit (立即启动)
        #    提供 /right_arm_controller/follow_joint_trajectory
        # ============================================
        ExecuteProcess(
            cmd=['bash', '-c',
                f'{ros2_env} && ros2 run g1arm_moveit g1arm_moveit'],
            cwd=ws_g1_dir,
            output='screen',
            name='g1arm_moveit'
        ),

        # ============================================
        # 3. bridge_dual_arm.py --hz 100 (立即启动)
        #    订阅 /lowstate，发布 /joint_states
        # ============================================
        ExecuteProcess(
            cmd=['bash', '-c',
                f'{ros2_env} && python3 bridge_dual_arm.py --hz 100'],
            cwd=home_dir,
            output='screen',
            name='bridge_dual_arm'
        ),
    ])
