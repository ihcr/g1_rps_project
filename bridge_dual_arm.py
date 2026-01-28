#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G1 双臂 14-DOF 桥接节点
- 订阅: /lowstate (unitree_hg/msg/LowState)
- 发布: /joint_states (sensor_msgs/JointState) - 包含左臂和右臂
用法示例：
python3 bridge_dual_arm.py --hz 100
"""

import math
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from unitree_hg.msg import LowState


def sensor_qos() -> QoSProfile:
    """与高频传感器话题匹配的 QoS。"""
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE
    return qos


class DualArmBridge(Node):
    def __init__(self, pub_hz: float):
        super().__init__('g1_dual_arm_bridge')

        # 左臂配置
        self.left_names = [
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
        ]
        self.left_indices = [15, 16, 17, 18, 19, 20, 21]
        self.left_signs = [1, 1, 1, 1, 1, 1, 1]

        # 右臂配置
        self.right_names = [
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        ]
        self.right_indices = [22, 23, 24, 25, 26, 27, 28]
        self.right_signs = [1, 1, 1, 1, 1, 1, 1]

        # 订阅 lowstate
        self._last = None
        self.create_subscription(LowState, '/lowstate', self._cb_lowstate, sensor_qos())

        # 发布 joint_states（合并左右臂）
        self._pub = self.create_publisher(JointState, '/joint_states', 10)

        # 定时发布频率
        self._timer = self.create_timer(max(1e-3, 1.0 / float(pub_hz)), self._tick)

        self.get_logger().info(
            f"Dual Arm Bridge ready.\n"
            f"  左臂: indices={self.left_indices}\n"
            f"  右臂: indices={self.right_indices}\n"
            f"  pub_hz = {pub_hz}"
        )

    def _cb_lowstate(self, msg: LowState):
        self._last = msg

    def _tick(self):
        if self._last is None:
            return

        ms = self._last.motor_state

        # 读取左臂位置
        left_pos = []
        for sgn, idx in zip(self.left_signs, self.left_indices):
            q = ms[idx].q if 0 <= idx < len(ms) else 0.0
            left_pos.append(sgn * q)

        # 读取右臂位置
        right_pos = []
        for sgn, idx in zip(self.right_signs, self.right_indices):
            q = ms[idx].q if 0 <= idx < len(ms) else 0.0
            right_pos.append(sgn * q)

        # 合并发布
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.left_names + self.right_names
        js.position = left_pos + right_pos
        # 可选：填充速度和力矩
        self._pub.publish(js)


def main():
    ap = argparse.ArgumentParser(description="G1 双臂 14-DOF /lowstate -> /joint_states 桥接")
    ap.add_argument('--hz', type=float, default=100.0, help='/joint_states 发布频率，默认 100Hz')
    args = ap.parse_args()

    rclpy.init()
    node = DualArmBridge(args.hz)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
