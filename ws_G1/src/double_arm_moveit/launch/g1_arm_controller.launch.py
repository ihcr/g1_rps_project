import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # 启动G1机械臂控制节点
        Node(
            package='g1arm_moveit',
            executable='g1arm_moveit_node',
            name='g1_arm_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            # 重映射话题以确保与Unitree G1驱动程序匹配
            remappings=[
                ('/lowcmd', '/lowcmd'),
                ('/lowstate', '/lowstate'),
                ('/left_arm/follow_joint_trajectory', '/left_arm_controller/follow_joint_trajectory'),
            ]
        ),
    ])