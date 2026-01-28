#!/usr/bin/env python3
"""
RealSense Image Bridge - ROS2 to Socket
订阅D435i的RGB图像话题，通过TCP socket发送JPEG压缩后的图像
"""
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

TCP_IP = "127.0.0.1"
TCP_PORT = 5006  # 使用不同的端口避免与UDP冲突

class RealSenseImageBridge(Node):
    def __init__(self):
        super().__init__("realsense_image_bridge")

        self.bridge = CvBridge()
        self.server_socket = None
        self.client_socket = None
        self.client_addr = None

        # 图像质量和帧率控制
        self.jpeg_quality = 80  # JPEG压缩质量 (0-100) - 提高质量，更快解码
        self.resize_scale = 0.75  # 缩放比例 - 提高分辨率以提升识别精度
        self.frame_count = 0
        self.last_send_time = time.time()
        self.fps = 0.0
        self.is_sending = False  # 发送中标志，用于跳帧
        self.dropped_frames = 0  # 丢弃帧计数

        # 订阅RealSense的RGB图像话题
        self.subscription = self.create_subscription(
            Image,
            '/g1_d435/color/image_raw',
            self.image_callback,
            10
        )

        # 设置TCP服务器
        self.setup_server()

        # 创建定时器等待客户端连接
        self.timer = self.create_timer(1.0, self.check_connection)

        self.get_logger().info(f'RealSense Image Bridge started on {TCP_IP}:{TCP_PORT}')
        self.get_logger().info('Waiting for client connection...')

    def setup_server(self):
        """设置TCP服务器"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # 低延迟优化：禁用Nagle算法
            self.server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # 大幅减小发送缓冲区避免积压（从65536降到32768）
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 32768)
            self.server_socket.bind((TCP_IP, TCP_PORT))
            self.server_socket.listen(1)
            self.server_socket.setblocking(False)  # 非阻塞模式
        except Exception as e:
            self.get_logger().error(f'Failed to setup server: {e}')

    def check_connection(self):
        """检查并接受客户端连接"""
        if self.client_socket is None and self.server_socket is not None:
            try:
                self.client_socket, self.client_addr = self.server_socket.accept()
                self.client_socket.setblocking(True)
                # 低延迟优化
                self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 32768)
                # 设置发送超时避免阻塞
                self.client_socket.settimeout(0.05)  # 50ms超时
                self.get_logger().info(f'Client connected from {self.client_addr}')
            except BlockingIOError:
                pass  # 没有新连接
            except Exception as e:
                self.get_logger().warn(f'Accept error: {e}')

    def image_callback(self, msg):
        """接收ROS图像并通过socket发送"""
        if self.client_socket is None:
            return

        # 跳帧机制：如果正在发送，丢弃当前帧
        if self.is_sending:
            self.dropped_frames += 1
            return

        self.is_sending = True
        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 降低分辨率以减少延迟
            if self.resize_scale < 1.0:
                new_width = int(cv_image.shape[1] * self.resize_scale)
                new_height = int(cv_image.shape[0] * self.resize_scale)
                cv_image = cv2.resize(cv_image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

            # JPEG压缩（快速模式）
            encode_param = [
                int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality,
                int(cv2.IMWRITE_JPEG_OPTIMIZE), 0  # 禁用优化加快编码
            ]
            result, encoded_image = cv2.imencode('.jpg', cv_image, encode_param)

            if not result:
                self.get_logger().warn('Failed to encode image')
                return

            # 获取压缩后的数据
            data = encoded_image.tobytes()
            size = len(data)

            # 获取当前时间戳
            timestamp = time.time()

            # 发送：时间戳(8字节double) + 大小(4字节) + 图像数据
            try:
                self.client_socket.sendall(struct.pack('>d', timestamp))  # 时间戳
                self.client_socket.sendall(struct.pack('>I', size))       # 大小
                self.client_socket.sendall(data)                          # 数据

                # 计算FPS
                self.frame_count += 1
                now = time.time()
                elapsed = now - self.last_send_time
                if elapsed >= 1.0:
                    self.fps = self.frame_count / elapsed
                    dropped_rate = self.dropped_frames / (self.frame_count + self.dropped_frames) * 100
                    self.frame_count = 0
                    self.dropped_frames = 0
                    self.last_send_time = now
                    self.get_logger().info(f'Streaming at {self.fps:.1f} FPS, size={size/1024:.1f}KB, dropped={dropped_rate:.1f}%')

            except (BrokenPipeError, ConnectionResetError) as e:
                self.get_logger().warn(f'Client disconnected: {e}')
                self.close_client()
            except socket.timeout:
                self.get_logger().warn('Send timeout - client too slow, dropping frame')
                # 不关闭连接，只是跳过这一帧
            except Exception as e:
                self.get_logger().error(f'Send error: {e}')
                self.close_client()

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')
        finally:
            self.is_sending = False

    def close_client(self):
        """关闭客户端连接"""
        if self.client_socket is not None:
            try:
                self.client_socket.close()
            except Exception:
                pass
            self.client_socket = None
            self.client_addr = None
            self.get_logger().info('Client connection closed. Waiting for reconnection...')

    def cleanup(self):
        """清理资源"""
        self.close_client()
        if self.server_socket is not None:
            try:
                self.server_socket.close()
            except Exception:
                pass

def main():
    rclpy.init()
    node = RealSenseImageBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
