#!/usr/bin/env python3
"""
RPS UDP to ROS2 桥接节点 (左手版本)

监听两个 UDP 端口：
1. 5015: AI 手势命令 (rock/paper/scissors) -> /rps/ai_move_left
2. 5017: 手臂控制命令 (raise/lower) -> /rps/arm_action_left
"""
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

UDP_IP = "127.0.0.1"
UDP_PORT_MOVE = 5015   # AI 手势命令端口 (左手)
UDP_PORT_ARM = 5017    # 手臂控制命令端口 (左手)

def normalize_cmd(raw: str) -> str:
    s = raw.strip().lower()
    if s in ("rock", "paper", "scissors"):
        return s
    # 兼容中文
    if s in ("石头", "拳头", "拳"):
        return "rock"
    if s in ("布",):
        return "paper"
    if s in ("剪刀",):
        return "scissors"
    return ""

def normalize_arm_cmd(raw: str) -> str:
    """规范化手臂控制命令"""
    s = raw.strip().lower()
    if s in ("raise", "up", "抬起"):
        return "raise"
    if s in ("lower", "down", "放下"):
        return "lower"
    return ""

class UdpToRosBridgeLeft(Node):
    def __init__(self):
        super().__init__("rps_udp_to_ros_bridge_left")

        # AI 手势命令发布者
        self.pub_move = self.create_publisher(String, "/rps/ai_move_left", 10)
        # 手臂控制命令发布者
        self.pub_arm = self.create_publisher(String, "/rps/arm_action_left", 10)

        # UDP socket for AI move commands
        self.sock_move = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_move.bind((UDP_IP, UDP_PORT_MOVE))
        self.sock_move.setblocking(False)

        # UDP socket for arm control commands
        self.sock_arm = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_arm.bind((UDP_IP, UDP_PORT_ARM))
        self.sock_arm.setblocking(False)

        self.last_cmd = ""
        self.last_time_ns = 0

        self.timer = self.create_timer(0.02, self.poll_udp)  # 50Hz
        self.get_logger().info(f"UDP listen {UDP_IP}:{UDP_PORT_MOVE} -> ROS2 /rps/ai_move_left")
        self.get_logger().info(f"UDP listen {UDP_IP}:{UDP_PORT_ARM} -> ROS2 /rps/arm_action_left")

    def poll_udp(self):
        # 处理 AI 手势命令
        self._poll_move_socket()
        # 处理手臂控制命令
        self._poll_arm_socket()

    def _poll_move_socket(self):
        """处理 AI 手势命令 UDP"""
        try:
            data, addr = self.sock_move.recvfrom(1024)
        except BlockingIOError:
            return

        raw = data.decode("utf-8", errors="ignore")
        cmd = normalize_cmd(raw)
        if not cmd:
            self.get_logger().warn(f"Unknown move msg='{raw}' from {addr}, ignored.")
            return

        # 去抖：0.3s 内重复同指令忽略
        now_ns = self.get_clock().now().nanoseconds
        if cmd == self.last_cmd and (now_ns - self.last_time_ns) < int(0.3 * 1e9):
            return
        self.last_cmd = cmd
        self.last_time_ns = now_ns

        msg = String()
        msg.data = cmd
        self.pub_move.publish(msg)
        self.get_logger().info(f"Published /rps/ai_move_left: {cmd} (from UDP {addr})")

    def _poll_arm_socket(self):
        """处理手臂控制命令 UDP"""
        try:
            data, addr = self.sock_arm.recvfrom(1024)
        except BlockingIOError:
            return

        raw = data.decode("utf-8", errors="ignore")
        cmd = normalize_arm_cmd(raw)
        if not cmd:
            self.get_logger().warn(f"Unknown arm msg='{raw}' from {addr}, ignored.")
            return

        msg = String()
        msg.data = cmd
        self.pub_arm.publish(msg)
        self.get_logger().info(f"Published /rps/arm_action_left: {cmd} (from UDP {addr})")

def main():
    rclpy.init()
    node = UdpToRosBridgeLeft()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.sock_move.close()
            node.sock_arm.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
