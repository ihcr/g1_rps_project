#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RPS 左臂控制节点 - 模拟石头剪刀布时的手臂摆动 (左手备用版本)

订阅: /rps/arm_action_left (std_msgs/String) - "raise" 或 "lower"
发布: /rps/ai_move_left (std_msgs/String) - 抬起手臂时发送 "rock" 让夹爪握拳
通过: /left_arm_controller/follow_joint_trajectory Action 控制左臂

使用方法：
    python3 rps_arm_controller_left.py

需要先运行：
    - ros2 run g1arm_moveit g1arm_moveit
    - python3 bridge_dual_arm.py --hz 100
"""

import sys
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from builtin_interfaces.msg import Duration


class RPSArmControllerLeft(Node):
    """RPS 左臂控制节点"""

    def __init__(self):
        super().__init__('rps_arm_controller_left')

        # 左臂关节名称
        self.joint_names = [
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
        ]

        # 初始位置（所有关节归零）
        self.raise_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # 抬起位置：左臂向上抬起约30度
        # 注意：左臂的 ShoulderPitch 方向与右臂相反
        self.init_pos = [
            -math.pi / 6,     # ShoulderPitch: 向上抬起约30度
            0.0,              # ShoulderRoll: 保持
            0.0,              # ShoulderYaw: 保持
            0.0,              # Elbow: 保持
            0.0,              # WristRoll: 保持
            0.0,              # WristPitch: 保持
            0.0,              # WristYaw: 保持
        ]

        # 动作持续时间（秒）
        self.raise_duration = 1.5   # 抬起用时
        self.lower_duration = 0.8   # 放下用时（快一点，模拟出拳）

        # 当前手臂状态
        self.arm_raised = False
        self.action_in_progress = False
        self.action_lock = threading.Lock()

        # 回调组（允许并发）
        self.callback_group = ReentrantCallbackGroup()

        # 创建 Action 客户端
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )

        # 订阅关节状态
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )

        # 订阅手臂控制命令
        self.arm_action_sub = self.create_subscription(
            String,
            '/rps/arm_action_left',
            self.arm_action_callback,
            10,
            callback_group=self.callback_group
        )

        # 发布夹爪控制命令（用于在抬起手臂时让夹爪握拳）
        self.gripper_pub = self.create_publisher(String, '/rps/ai_move_left', 10)

        self.get_logger().info('RPS 左臂控制节点初始化中...')

        # 等待 Action Server
        self.server_ready = False
        self.init_thread = threading.Thread(target=self._wait_for_server, daemon=True)
        self.init_thread.start()

    def _wait_for_server(self):
        """后台线程等待 Action Server"""
        self.get_logger().info('等待连接到 Action 服务器...')
        if self.action_client.wait_for_server(timeout_sec=30.0):
            self.server_ready = True
            self.get_logger().info('已连接到 Action 服务器！')
            self.get_logger().info('等待 /rps/arm_action_left 命令...')
            self.get_logger().info('  - "raise": 抬起左臂30度')
            self.get_logger().info('  - "lower": 放回初始位置')
        else:
            self.get_logger().error('无法连接到 /left_arm_controller/follow_joint_trajectory')
            self.get_logger().error('请确保 g1arm_moveit 节点正在运行')

    def joint_state_callback(self, msg):
        """接收关节状态"""
        self.current_joint_state = msg

    def arm_action_callback(self, msg):
        """处理手臂控制命令"""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'收到命令: {cmd}')

        if not self.server_ready:
            self.get_logger().warn('Action 服务器尚未就绪，忽略命令')
            return

        if cmd == 'raise':
            self.raise_arm()
        elif cmd == 'lower':
            self.lower_arm()
        else:
            self.get_logger().warn(f'未知命令: {cmd}')

    def get_current_positions(self):
        """获取当前左臂关节位置"""
        if self.current_joint_state is None:
            return None

        positions = []
        for joint_name in self.joint_names:
            try:
                idx = self.current_joint_state.name.index(joint_name)
                positions.append(self.current_joint_state.position[idx])
            except (ValueError, IndexError):
                return None

        return positions

    def raise_arm(self):
        """抬起左臂"""
        if self.arm_raised:
            self.get_logger().info('手臂已经抬起，跳过')
            return

        with self.action_lock:
            if self.action_in_progress:
                self.get_logger().info('动作进行中，跳过')
                return
            self.action_in_progress = True

        # 同时让夹爪做出初始待机姿势（半张开，与石头剪刀布都不同）
        self._send_gripper_command("ready")

        # 在新线程中执行，避免阻塞
        thread = threading.Thread(
            target=self._execute_trajectory,
            args=(self.raise_pos, self.raise_duration, "抬起左臂", True),
            daemon=True
        )
        thread.start()

    def _send_gripper_command(self, gesture: str):
        """发送夹爪控制命令"""
        msg = String()
        msg.data = gesture
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'发送夹爪命令: {gesture}')

    def lower_arm(self):
        """放下左臂"""
        if not self.arm_raised:
            self.get_logger().info('手臂已经放下，跳过')
            return

        with self.action_lock:
            if self.action_in_progress:
                self.get_logger().info('动作进行中，跳过')
                return
            self.action_in_progress = True

        # 在新线程中执行
        thread = threading.Thread(
            target=self._execute_trajectory,
            args=(self.init_pos, self.lower_duration, "放下左臂", False),
            daemon=True
        )
        thread.start()

    def _execute_trajectory(self, target_positions, duration_sec, description, is_raise):
        """执行轨迹（在独立线程中）"""
        try:
            current_positions = self.get_current_positions()
            if current_positions is None:
                self.get_logger().error('无法获取当前关节位置')
                return

            self.get_logger().info(f'执行: {description}')

            # 创建轨迹
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names

            # 起点
            start_point = JointTrajectoryPoint()
            start_point.positions = current_positions
            start_point.time_from_start = Duration(sec=0, nanosec=50_000_000)  # 0.05s

            # 终点
            end_point = JointTrajectoryPoint()
            end_point.positions = target_positions
            # 处理小数秒
            sec = int(duration_sec)
            nanosec = int((duration_sec - sec) * 1e9)
            end_point.time_from_start = Duration(sec=sec, nanosec=nanosec)

            trajectory.points = [start_point, end_point]

            # 创建并发送 goal
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = trajectory

            future = self.action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error(f'{description} - Goal 被拒绝')
                return

            # 等待结果
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 2.0)

            result = result_future.result()
            if result and result.status == 4:  # SUCCEEDED
                self.get_logger().info(f'{description} - 完成')
                self.arm_raised = is_raise
            else:
                self.get_logger().warn(f'{description} - 状态码: {result.status if result else "None"}')

        except Exception as e:
            self.get_logger().error(f'{description} - 异常: {e}')
        finally:
            with self.action_lock:
                self.action_in_progress = False


def main(args=None):
    rclpy.init(args=args)

    node = RPSArmControllerLeft()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，退出')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
