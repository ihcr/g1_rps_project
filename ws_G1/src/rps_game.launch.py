#!/usr/bin/env python3
"""
RPS 游戏应用层 Launch 文件

启动内容：
1. realsense_image_bridge.py (相机图像桥接)
2. ros_bridge.py (UDP -> ROS2 话题桥接)
3. rps_arm_controller.py (RPS手臂摆动控制)

注意：
- 需要先启动 g1_robot_base.launch.py
- test6_pro_socket_cheat_detect.py 需要单独启动（因为需要conda环境）

使用方法：
    # 先设置环境变量
    source /opt/ros/humble/setup.bash
    source ~/unitree_ros2/setup.sh
    source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID=0
    source ~/ws_G1/install/setup.bash

    # 启动
    ros2 launch ~/ws_G1/src/rps_game.launch.py

    # 然后在另一个终端手动启动手势识别（需要conda）：
    cd ~/UCL/Project/rock-paper-scissors
    conda activate rock_paper
    python3 test6_pro_socket_cheat_detect.py
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    home_dir = os.path.expanduser('~')
    rps_dir = os.path.join(home_dir, 'UCL/Project/rock-paper-scissors')
    ws_g1_src_dir = os.path.join(home_dir, 'ws_G1/src')

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
        # 2. ros_bridge.py (延迟1秒)
        #    监听 UDP:5005 (AI手势) -> /rps/ai_move
        #    监听 UDP:5007 (手臂控制) -> /rps/arm_action
        # ============================================
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c',
                        f'{ros2_env} && python3 ros_bridge.py'],
                    cwd=rps_dir,
                    output='screen',
                    name='ros_bridge'
                ),
            ]
        ),

        # ============================================
        # 3. rps_arm_controller.py (延迟2秒)
        #    订阅 /rps/arm_action，控制右臂摆动
        #    需要 g1arm_moveit 的 Action Server
        # ============================================
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c',
                        f'{ros2_env} && python3 rps_arm_controller.py'],
                    cwd=ws_g1_src_dir,
                    output='screen',
                    name='rps_arm_controller'
                ),
            ]
        ),
    ])
