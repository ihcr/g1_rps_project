#!/usr/bin/env python3
"""
RPS 游戏应用层 Launch 文件 (左手版本)

启动内容：
1. realsense_image_bridge.py (相机图像桥接)
2. ros_bridge_left.py (UDP -> ROS2 话题桥接，左手版本)
3. rps_arm_controller_left.py (RPS左臂摆动控制)

注意：
- 需要先启动 g1_robot_base_left.launch.py
- test2_3_combined_left.py 需要单独启动（因为需要conda环境）

使用方法：
    # 先设置环境变量
    source /opt/ros/humble/setup.bash
    source ~/unitree_ros2/setup.sh
    source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=0
    source ~/g1_rps_project/ws_G1/install/setup.bash

    # 启动
    ros2 launch ~/g1_rps_project/ws_G1/src/rps_game_left.launch.py

    # 然后在另一个终端手动启动手势识别（需要conda）：
    cd ~/g1_rps_project/rock-paper-scissors
    conda activate rock_paper
    python3 test2_3_combined_left.py
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    home_dir = os.path.expanduser('~')
    project_dir = os.path.join(home_dir, 'g1_rps_project')
    rps_dir = os.path.join(project_dir, 'rock-paper-scissors')
    ws_g1_src_dir = os.path.join(project_dir, 'ws_G1/src')
    ws_g1_dir = os.path.join(project_dir, 'ws_G1')

    # ROS2 环境设置命令
    ros2_env = (
        'source /opt/ros/humble/setup.bash && '
        'source ~/unitree_ros2/setup.sh && '
        'source ~/unitree_ros2/cyclonedds_ws/install/setup.bash && '
        'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && '
        'export ROS_DOMAIN_ID=0 && '
        f'source {ws_g1_dir}/install/setup.bash'
    )

    return LaunchDescription([
        # ============================================
        # 1. realsense_image_bridge.py (立即启动)
        #    订阅 /g1_d435/color/image_raw
        #    通过TCP:5006发送图像给手势识别程序
        # ============================================
        ExecuteProcess(
            cmd=['bash', '-c',
                f'{ros2_env} && python3 realsense_image_bridge.py'],
            cwd=rps_dir,
            output='screen',
            name='realsense_image_bridge'
        ),

        # ============================================
        # 2. ros_bridge_left.py (延迟1秒)
        #    监听 UDP:5015 (AI手势) -> /rps/ai_move_left
        #    监听 UDP:5017 (手臂控制) -> /rps/arm_action_left
        # ============================================
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c',
                        f'{ros2_env} && python3 ros_bridge_left.py'],
                    cwd=rps_dir,
                    output='screen',
                    name='ros_bridge_left'
                ),
            ]
        ),

        # ============================================
        # 3. rps_arm_controller_left.py (延迟2秒)
        #    订阅 /rps/arm_action_left，控制左臂摆动
        #    需要 g1arm_moveit 的 Action Server
        # ============================================
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c',
                        f'{ros2_env} && python3 rps_arm_controller_left.py'],
                    cwd=ws_g1_src_dir,
                    output='screen',
                    name='rps_arm_controller_left'
                ),
            ]
        ),
    ])
