"""
纯启发式版本 (Socket Bridge版本) - 作弊检测版 + 视觉区域优化
主要特点：
1. **完全去掉CNN模型，只使用启发式规则判断**
2. 基于手指伸展状态的几何特征检测
3. 通过Socket从RealSense bridge接收图像
4. 结合Markov预测模型做AI决策
5. **NEW: 在锁定期间检测手势变化，发现作弊行为**
6. **NEW: 限制手部检测区域为画面右侧，用绿色框显示**
"""

import time
from collections import deque
import socket
import struct
import subprocess
import threading

import cv2
import mediapipe as mp
import numpy as np
# from keras.models import load_model  # 不再需要CNN模型

# =========================
# Socket Image Receiver
# =========================
class ImageReceiver:
    def __init__(self, host='127.0.0.1', port=5006):
        self.host = host
        self.port = port
        self.sock = None
        self.connected = False
        self.max_frame_age = 0.1  # 最大帧年龄（秒），超过此时间的帧将被丢弃
        self.dropped_old_frames = 0
        self.received_frames = 0

    def connect(self):
        """连接到图像bridge"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # 低延迟优化
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # 大幅减小接收缓冲区避免积压（从65536降到16384）
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16384)
            # 设置超时避免阻塞（减小到50ms）
            self.sock.settimeout(0.05)
            self.sock.connect((self.host, self.port))
            self.connected = True
            print(f"[ImageReceiver] Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"[ImageReceiver] Connection failed: {e}")
            return False

    def receive_image(self):
        """接收一帧图像（带时间戳检查）"""
        if not self.connected:
            return None

        try:
            # 接收8字节的时间戳
            timestamp_data = self._recv_all(8)
            if timestamp_data is None:
                return None

            frame_timestamp = struct.unpack('>d', timestamp_data)[0]

            # 接收4字节的图像大小
            size_data = self._recv_all(4)
            if size_data is None:
                return None

            size = struct.unpack('>I', size_data)[0]

            # 接收图像数据
            img_data = self._recv_all(size)
            if img_data is None:
                return None

            # 检查帧的年龄
            now = time.time()
            frame_age = now - frame_timestamp

            self.received_frames += 1

            # 如果帧太旧，丢弃它
            if frame_age > self.max_frame_age:
                self.dropped_old_frames += 1
                if self.received_frames % 30 == 0:  # 每30帧打印一次统计
                    drop_rate = self.dropped_old_frames / self.received_frames * 100
                    print(f"[ImageReceiver] Frame age {frame_age*1000:.1f}ms > {self.max_frame_age*1000:.1f}ms, dropped. Drop rate: {drop_rate:.1f}%")
                return None

            # 解码JPEG
            img_array = np.frombuffer(img_data, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            return frame

        except socket.timeout:
            # 超时是正常的，不打印错误
            return None
        except Exception as e:
            print(f"[ImageReceiver] Error receiving image: {e}")
            self.connected = False
            return None

    def _recv_all(self, size):
        """接收指定大小的数据"""
        data = b''
        while len(data) < size:
            try:
                packet = self.sock.recv(size - len(data))
                if not packet:
                    return None
                data += packet
            except socket.timeout:
                # 超时时返回None，让上层处理
                return None
            except Exception as e:
                print(f"[ImageReceiver] Receive error: {e}")
                return None
        return data

    def close(self):
        """关闭连接"""
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.connected = False


# =========================
# UDP Sender
# =========================
UDP_IP = "127.0.0.1"
UDP_PORT = 5005       # AI 手势命令端口
UDP_PORT_ARM = 5007   # 手臂控制命令端口
_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_udp_sock_arm = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_ai_move(ai_move_name: str):
    msg = ai_move_name.strip().lower().encode("utf-8")
    _udp_sock.sendto(msg, (UDP_IP, UDP_PORT))
    print(f"[UDP] Sent AI move -> {ai_move_name}")

def send_arm_command(cmd: str):
    """发送手臂控制命令: 'raise' 或 'lower'"""
    msg = cmd.strip().lower().encode("utf-8")
    _udp_sock_arm.sendto(msg, (UDP_IP, UDP_PORT_ARM))
    print(f"[ARM] Sent arm command -> {cmd}")

# =========================
# TTS (Text-to-Speech) Function - G1 Robot
# =========================
import os
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
G1_SPEAK_PATH = os.path.join(SCRIPT_DIR, "g1_speak.sh")  # 使用 shell 脚本

def speak_async(text):
    """异步播放TTS，不阻塞主线程
    优先使用 G1 机器人的 TTS，失败则使用电脑的 espeak
    """
    def _speak():
        try:
            # 方案1: 尝试使用 G1 机器人的 TTS
            if os.path.exists(G1_SPEAK_PATH):
                print(f"[TTS] Using G1 robot TTS: {text}")
                result = subprocess.run([G1_SPEAK_PATH, text],
                                      capture_output=True, timeout=5)
                if result.returncode == 0:
                    print(f"[TTS] G1 TTS success")
                    return
                else:
                    print(f"[TTS] G1 TTS failed: {result.stderr.decode()}")

            # 方案2: 回退到电脑扬声器 (espeak)
            print(f"[TTS] Falling back to computer speaker (espeak)")
            subprocess.run(['espeak', text, '-s', '150', '-v', 'en+f3'],
                          check=True, capture_output=True, timeout=3)
            print(f"[TTS] espeak success")
        except Exception as e:
            print(f"[TTS] All TTS methods failed: {e}")

    # 在后台线程播放，避免阻塞游戏
    thread = threading.Thread(target=_speak, daemon=True)
    thread.start()

# MediaPipe初始化
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.4,
    min_tracking_confidence=0.4,
)

# 加载模型 (已禁用 - 只使用启发式规则)
# model = load_model("model/rock_paper_scissors_model.h5")

REV_CLASS_MAP = {0: "paper", 1: "rock", 2: "scissors"}
MOVE_TO_IDX = {"paper": 0, "rock": 1, "scissors": 2}
IDX_TO_MOVE = {v: k for k, v in MOVE_TO_IDX.items()}

# Markov模型
trans_counts = np.ones((3, 3), dtype=np.float32)
base_counts  = np.ones(3, dtype=np.float32)
prev_locked_move = None

FUSE_EPS = 1e-6
FUSE_ALPHA_MIN = 0.35
FUSE_ALPHA_MAX = 0.95
PRED_LAMBDA = 0.75

_PAYOFF = np.array([
    [ 0,  1, -1],
    [-1,  0,  1],
    [ 1, -1,  0],
], dtype=np.float32)

def _pred_dist_mixed(prev_move_name, lam: float = PRED_LAMBDA):
    p_uni = base_counts / (base_counts.sum() + 1e-12)
    if prev_move_name in MOVE_TO_IDX:
        row = trans_counts[MOVE_TO_IDX[prev_move_name]]
        p_markov = row / (row.sum() + 1e-12)
        p = lam * p_markov + (1.0 - lam) * p_uni
    else:
        p = p_uni
    p = p.astype(np.float32)
    return p / (p.sum() + 1e-12)

def _fuse_probs(p_vis, p_pred, alpha):
    p_vis = np.asarray(p_vis, dtype=np.float32)
    p_pred = np.asarray(p_pred, dtype=np.float32)
    p = np.power(p_vis + FUSE_EPS, alpha) * np.power(p_pred + FUSE_EPS, 1.0 - alpha)
    return p / (p.sum() + 1e-12)

def _choose_ai_move_from_belief(p_user):
    p_user = np.asarray(p_user, dtype=np.float32).reshape(3,)
    scores = _PAYOFF @ p_user
    ai_idx = int(np.argmax(scores))
    return IDX_TO_MOVE[ai_idx]

ai_images = {
    "rock": cv2.imread("images/rock.png"),
    "paper": cv2.imread("images/paper.png"),
    "scissors": cv2.imread("images/scissors.png"),
}

def mapper(val: int) -> str:
    return REV_CLASS_MAP.get(val, "unknown")

def preprocess_landmarks(landmarks, img_size=128):
    """
    改进版：添加手指骨骼连线
    """
    xs = [lm.x for lm in landmarks]
    ys = [lm.y for lm in landmarks]

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)

    x_center = (x_min + x_max) / 2.0
    y_center = (y_min + y_max) / 2.0
    bbox_size = max(x_max - x_min, y_max - y_min)
    if bbox_size < 1e-6:
        bbox_size = 1e-6

    scale = img_size / bbox_size
    black_bg = np.zeros((img_size, img_size, 3), dtype=np.uint8)

    # 转换坐标
    points = []
    for lm in landmarks:
        x = int((lm.x - x_center) * scale + img_size / 2)
        y = int((lm.y - y_center) * scale + img_size / 2)
        points.append((x, y))

    # ===== 新增：画手指骨骼连线 =====
    # MediaPipe Hand connections
    connections = [
        (0,1),(1,2),(2,3),(3,4),        # 大拇指
        (0,5),(5,6),(6,7),(7,8),        # 食指
        (0,9),(9,10),(10,11),(11,12),   # 中指
        (0,13),(13,14),(14,15),(15,16), # 无名指
        (0,17),(17,18),(18,19),(19,20), # 小指
        (5,9),(9,13),(13,17)            # 手掌横向连接
    ]

    # 先画线（细一点）
    for start_idx, end_idx in connections:
        cv2.line(black_bg, points[start_idx], points[end_idx], (255, 255, 255), 1)

    # 再画点（粗一点，覆盖线的端点）
    for pt in points:
        cv2.circle(black_bg, pt, 2, (255, 255, 255), -1)

    return black_bg


# ===== 新增：启发式规则检测 =====
def heuristic_gesture_check(landmarks):
    """
    基于几何特征判断手势类型
    返回: (gesture_name, confidence)
    """
    # MediaPipe手指关键点索引
    WRIST = 0
    THUMB_TIP = 4
    INDEX_TIP = 8
    INDEX_PIP = 6
    MIDDLE_TIP = 12
    MIDDLE_PIP = 10
    RING_TIP = 16
    RING_PIP = 14
    PINKY_TIP = 20
    PINKY_PIP = 18

    # 提取3D坐标（使用x, y, z）
    def get_point(idx):
        return np.array([landmarks[idx].x, landmarks[idx].y, landmarks[idx].z])

    wrist = get_point(WRIST)

    # 计算每根手指的伸展程度
    def is_extended(tip_idx, pip_idx):
        """判断手指是否伸展：tip到手腕的距离 > pip到手腕的距离"""
        tip = get_point(tip_idx)
        pip = get_point(pip_idx)
        dist_tip = np.linalg.norm(tip - wrist)
        dist_pip = np.linalg.norm(pip - wrist)
        return dist_tip > dist_pip * 1.1  # 1.1是容差系数

    thumb_ext = is_extended(THUMB_TIP, 2)  # 大拇指用第2个关节
    index_ext = is_extended(INDEX_TIP, INDEX_PIP)
    middle_ext = is_extended(MIDDLE_TIP, MIDDLE_PIP)
    ring_ext = is_extended(RING_TIP, RING_PIP)
    pinky_ext = is_extended(PINKY_TIP, PINKY_PIP)

    extended_count = sum([thumb_ext, index_ext, middle_ext, ring_ext, pinky_ext])

    # ===== 判断逻辑 =====

    # 石头：所有手指都收拢
    if extended_count == 0 or extended_count == 1:  # 允许1根手指稍微伸展（容差）
        return "rock", 0.85

    # 布：大部分手指伸展
    if extended_count >= 4:
        return "paper", 0.85

    # 剪刀：食指和中指伸展，其他收拢
    if index_ext and middle_ext and not ring_ext and not pinky_ext:
        return "scissors", 0.90  # 高置信度

    # 剪刀变体：只要食指中指伸展就算（容差）
    if index_ext and middle_ext:
        return "scissors", 0.75

    # 剪刀变体2：两根手指伸展（可能是食指中指）
    if extended_count == 2:
        # 检查是否是食指+中指的组合
        if index_ext or middle_ext:
            return "scissors", 0.65

    # 无法确定
    return None, 0.0


def fuse_cnn_and_heuristic(p_cnn, heur_gesture, heur_conf, weight=0.4):
    """
    融合CNN预测和启发式规则
    weight: 启发式规则的权重（0-1）
    """
    p_result = p_cnn.copy()

    if heur_gesture is not None and heur_conf > 0.5:
        # 创建启发式概率分布
        p_heur = np.array([0.05, 0.05, 0.05], dtype=np.float32)
        heur_idx = MOVE_TO_IDX[heur_gesture]
        p_heur[heur_idx] = heur_conf

        # 归一化
        p_heur = p_heur / p_heur.sum()

        # 融合
        p_result = (1 - weight) * p_cnn + weight * p_heur
        p_result = p_result / p_result.sum()

    return p_result


# 运动检测参数
MOVE_START_TH = 0.006
MOVE_END_TH = 0.005
END_STILL_FRAMES = 1
MOVE_HISTORY = 2
LOCK_HOLD_SEC = 2.0

USE_DECELERATION = True
DECEL_RATIO = 0.6
MIN_DECISION_FRAMES = 3

# ===== 新增：启发式规则权重 =====
USE_HEURISTIC = True       # 是否启用启发式规则
HEURISTIC_WEIGHT = 0.3     # 启发式权重（0.3 = 30%启发式 + 70%CNN）

# ===== 新增：作弊检测参数 =====
CHEAT_DETECTION_ENABLED = True  # 是否启用作弊检测
CHEAT_CONFIDENCE_THRESHOLD = 0.70  # 手势置信度阈值（超过此值才判断为手势变化）
CHEAT_WARNING_DURATION = 1.5  # 作弊警告显示时长（秒）

# ===== 新增：检测区域限制参数 =====
DETECTION_ROI_ENABLED = True  # 是否启用检测区域限制
DETECTION_ROI_WIDTH_RATIO = 0.4  # ROI 宽度占画面的 40%
DETECTION_ROI_START_RATIO = 0.4  # ROI 从图像宽度的 50% 处开始（可调整此值左右移动框）

# 队列和状态
move_q = deque(maxlen=MOVE_HISTORY)
decision_q = deque(maxlen=50)

in_motion = False
still_count = 0
prev_wrist = None
locked_until = 0.0
peak_move = 0.0

user_move_name = None
ai_move_name = None

# ===== 新增：作弊检测相关变量 =====
locked_gesture = None  # 锁定时的手势
cheat_detected = False  # 是否检测到作弊
cheat_warning_until = 0.0  # 作弊警告显示到什么时候

# ===== 新增：手臂控制相关变量 =====
arm_raised = False  # 手臂是否已抬起
ARM_RAISE_DELAY = 0.8  # 出拳后多久开始抬起手臂（秒）
arm_raise_scheduled_at = 0.0  # 计划抬起手臂的时间

dbg_vis_name = None
dbg_vis_conf = None
dbg_heur_name = None
dbg_heur_conf = None
dbg_final_name = None
dbg_final_conf = None

lock_events = 0
lock_vis_low_90 = 0
lock_vis_low_95 = 0

# 创建图像接收器
image_receiver = ImageReceiver()

print("=== Heuristic-Only Version with Cheat Detection (Socket Bridge) ===")
print("=== Visual Optimisation: Detection ROI Enabled ===")
print("Using ONLY heuristic rules for gesture recognition")
print("Cheat detection ENABLED - will monitor gesture changes during lock period")
print("Arm swing simulation ENABLED - arm will raise/lower during RPS")
if DETECTION_ROI_ENABLED:
    print(f"Detection ROI ENABLED - only detecting hands in right {int((1 - DETECTION_ROI_START_RATIO) * 100)}% of frame")
print("Connecting to RealSense image bridge...")

# 连接到bridge
if not image_receiver.connect():
    print("Failed to connect to image bridge. Please start realsense_image_bridge.py first!")
    exit(1)

# ===== 启动时抬起手臂 =====
print("[ARM] Raising arm at startup...")
send_arm_command("raise")
arm_raised = True

try:
    frame_count = 0
    last_time = time.time()
    no_frame_count = 0

    while True:
        # 从socket接收图像
        frame = image_receiver.receive_image()

        if frame is None:
            no_frame_count += 1
            # 只在连续多次无帧后才尝试重连
            if no_frame_count > 20:  # 约1秒（假设循环很快）
                print("No frame received for 1 second, trying to reconnect...")
                time.sleep(1)
                if not image_receiver.connect():
                    print("Reconnection failed!")
                    break
                no_frame_count = 0
            continue

        no_frame_count = 0  # 重置计数器

        frame_count += 1
        now = time.time()
        locked = now < locked_until

        # ===== 新增：检查是否需要抬起手臂 =====
        if arm_raise_scheduled_at > 0 and now >= arm_raise_scheduled_at:
            if not arm_raised:
                send_arm_command("raise")
                arm_raised = True
            arm_raise_scheduled_at = 0.0  # 重置计划时间

        frame = cv2.flip(frame, 1)
        height, width, _ = frame.shape
        half_width = width // 2

        # ===== 检测区域限制 =====
        if DETECTION_ROI_ENABLED:
            # 计算 ROI 区域（固定宽度，可调整位置）
            roi_width = int(width * DETECTION_ROI_WIDTH_RATIO)
            roi_x_start = int(width * DETECTION_ROI_START_RATIO)
            roi_x_end = min(roi_x_start + roi_width, width)  # 确保不超出右边界

            # 提取 ROI 区域进行处理
            roi = frame[:, roi_x_start:roi_x_end]
            image_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
            results = hands.process(image_rgb)

            # 绘制绿色检测区域框
            cv2.rectangle(frame, (roi_x_start, 0), (roi_x_end - 1, height - 1), (0, 255, 0), 2)
        else:
            # 不限制区域，处理整个画面
            roi_x_start = 0
            roi_width = width
            image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(image_rgb)

        live_vis_name = None
        live_vis_conf = None
        live_heur_name = None
        live_heur_conf = None
        live_move_name = None
        live_conf = None
        live_p_final = None
        mean_move = None
        status_msg = "Ready"

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            landmarks = hand_landmarks.landmark

            # 1) 运动检测
            # 将 ROI 内的归一化坐标转换为整个图像的归一化坐标
            if DETECTION_ROI_ENABLED:
                wrist_x_in_full = (roi_x_start + landmarks[0].x * roi_width) / width
                wrist_xy = (wrist_x_in_full, landmarks[0].y)
            else:
                wrist_xy = (landmarks[0].x, landmarks[0].y)
            if prev_wrist is not None:
                move = np.linalg.norm(np.array(wrist_xy) - np.array(prev_wrist))
                move_q.append(move)
                mean_move = float(np.mean(move_q)) if len(move_q) else move
            prev_wrist = wrist_xy

            # 2) 启发式检测 (仅使用启发式规则，不使用CNN)
            heur_gesture, heur_conf = heuristic_gesture_check(landmarks)
            live_heur_name = heur_gesture
            live_heur_conf = heur_conf

            # 3) 直接使用启发式规则生成视觉概率分布
            if heur_gesture is not None and heur_conf > 0.5:
                # 创建基于启发式的概率分布
                p_vis = np.array([0.05, 0.05, 0.05], dtype=np.float32)
                heur_idx = MOVE_TO_IDX[heur_gesture]
                p_vis[heur_idx] = heur_conf
                # 归一化
                p_vis = p_vis / p_vis.sum()
            else:
                # 如果启发式无法判断，使用均匀分布
                p_vis = np.array([1.0/3, 1.0/3, 1.0/3], dtype=np.float32)

            vis_top = int(np.argmax(p_vis))
            live_vis_name = mapper(vis_top)
            live_vis_conf = float(p_vis[vis_top])

            # 4) 预测分布
            p_pred = _pred_dist_mixed(prev_locked_move)

            # 5) 融合视觉与预测
            alpha = float(np.clip(live_vis_conf, FUSE_ALPHA_MIN, FUSE_ALPHA_MAX))
            p_final = _fuse_probs(p_vis, p_pred, alpha=alpha)
            final_top = int(np.argmax(p_final))
            live_move_name = mapper(final_top)
            live_conf = float(p_final[final_top])
            live_p_final = p_final

            dbg_vis_name = live_vis_name
            dbg_vis_conf = live_vis_conf
            dbg_heur_name = live_heur_name
            dbg_heur_conf = live_heur_conf
            dbg_final_name = live_move_name
            dbg_final_conf = live_conf

            # 6) 运动检测逻辑
            if not locked:
                decision_q.append({
                    "t": now,
                    "p_vis": p_vis,
                    "vis_top": vis_top,
                    "vis_conf": live_vis_conf,
                    "p_final": p_final
                })

                if mean_move is not None:
                    if not in_motion:
                        if mean_move >= MOVE_START_TH:
                            in_motion = True
                            still_count = 0
                            peak_move = mean_move
                            decision_q.clear()
                            decision_q.append({
                                "t": now,
                                "p_vis": p_vis,
                                "vis_top": vis_top,
                                "vis_conf": live_vis_conf,
                                "p_final": p_final
                            })
                            status_msg = "Motion detected..."
                    else:
                        if mean_move > peak_move:
                            peak_move = mean_move

                        below_threshold = mean_move <= MOVE_END_TH
                        is_decelerating = (USE_DECELERATION and
                                          peak_move > MOVE_START_TH and
                                          mean_move < peak_move * DECEL_RATIO)

                        if below_threshold or is_decelerating:
                            still_count += 1
                        else:
                            still_count = 0

                        status_msg = f"Motion... (still={still_count}/{END_STILL_FRAMES})"

                        has_enough_frames = len(decision_q) >= MIN_DECISION_FRAMES
                        if still_count >= END_STILL_FRAMES and has_enough_frames:
                            best = max(decision_q, key=lambda d: d["vis_conf"])
                            user_idx = int(best["vis_top"])
                            user_move_name = mapper(user_idx)

                            base_counts[user_idx] += 1.0
                            if prev_locked_move in MOVE_TO_IDX:
                                prev_idx = MOVE_TO_IDX[prev_locked_move]
                                trans_counts[prev_idx, user_idx] += 1.0
                            prev_locked_move = user_move_name

                            ai_move_name = _choose_ai_move_from_belief(best["p_final"])
                            print(f"User: {user_move_name}, AI: {ai_move_name}")

                            lock_events += 1
                            lock_conf = float(best["vis_conf"])
                            if lock_conf < 0.90:
                                lock_vis_low_90 += 1
                            if lock_conf < 0.95:
                                lock_vis_low_95 += 1

                            send_ai_move(ai_move_name)
                            locked_until = now + LOCK_HOLD_SEC

                            # ===== 新增：出拳时放下手臂 =====
                            if arm_raised:
                                send_arm_command("lower")
                                arm_raised = False
                                # 计划在一段时间后重新抬起手臂
                                arm_raise_scheduled_at = now + ARM_RAISE_DELAY

                            # ===== 新增：记录锁定时的手势 =====
                            locked_gesture = user_move_name
                            cheat_detected = False

                            in_motion = False
                            still_count = 0
                            peak_move = 0.0
                            decision_q.clear()
                            move_q.clear()
                            status_msg = "Locked"
                        elif still_count >= END_STILL_FRAMES and not has_enough_frames:
                            status_msg = f"Collecting frames {len(decision_q)}/{MIN_DECISION_FRAMES}"
            else:
                # ===== 新增：锁定期间的作弊检测逻辑 =====
                status_msg = "Locked"

                if CHEAT_DETECTION_ENABLED and locked_gesture is not None:
                    # 检测当前手势
                    if heur_gesture is not None and heur_conf > CHEAT_CONFIDENCE_THRESHOLD:
                        # 如果当前手势与锁定时的手势不同，判定为作弊
                        if heur_gesture != locked_gesture:
                            if not cheat_detected:
                                cheat_detected = True
                                cheat_warning_until = now + CHEAT_WARNING_DURATION
                                print(f"[CHEAT DETECTED] Gesture changed from {locked_gesture} to {heur_gesture}!")
                                # ===== 新增：播放语音警告 =====
                                speak_async("You cheated!")
                            status_msg = "CHEAT DETECTED!"

        else:
            prev_wrist = None
            if not locked:
                in_motion = False
                still_count = 0
                peak_move = 0.0
                move_q.clear()
                decision_q.clear()
                user_move_name = None
                ai_move_name = None
                status_msg = "No hand detected."
            else:
                status_msg = "Locked"

        # 显示 - 完整画面 + AI出拳图片
        ai_panel_width = width // 3  # AI面板宽度为画面宽度的1/3
        if ai_move_name and ai_move_name in ai_images and ai_images[ai_move_name] is not None:
            ai_move_display = cv2.resize(ai_images[ai_move_name], (ai_panel_width, height))
        else:
            ai_move_display = np.zeros((height, ai_panel_width, 3), dtype=np.uint8)

        combined_display = np.hstack((frame, ai_move_display))
        total_width = width + ai_panel_width  # 组合后的总宽度

        cv2.putText(combined_display, status_msg, (10, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

        # 识别信息
        y0 = 30
        dy = 28

        show_vis_name = live_vis_name if live_vis_name is not None else dbg_vis_name
        show_vis_conf = live_vis_conf if live_vis_conf is not None else dbg_vis_conf
        show_heur_name = live_heur_name if live_heur_name is not None else dbg_heur_name
        show_heur_conf = live_heur_conf if live_heur_conf is not None else dbg_heur_conf
        show_final_name = live_move_name if live_move_name is not None else dbg_final_name
        show_final_conf = live_conf if live_conf is not None else dbg_final_conf

        if show_vis_name is not None and show_vis_conf is not None:
            cv2.putText(combined_display, f"CNN: {show_vis_name} ({show_vis_conf:.2f})",
                        (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA)

        if show_heur_name is not None and show_heur_conf is not None:
            cv2.putText(combined_display, f"HEUR: {show_heur_name} ({show_heur_conf:.2f})",
                        (10, y0 + dy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 150, 0), 2, cv2.LINE_AA)

        if show_final_name is not None and show_final_conf is not None:
            cv2.putText(combined_display, f"FINAL: {show_final_name} ({show_final_conf:.2f})",
                        (10, y0 + 2*dy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

        if mean_move is not None:
            cv2.putText(combined_display, f"Move: {mean_move:.4f}",
                        (10, y0 + 3*dy), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)

        if ai_move_name:
            cv2.putText(combined_display, f"AI: {ai_move_name}",
                        (width + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)

        # ===== 新增：显示作弊警告 =====
        if now < cheat_warning_until:
            warning_text = "YOU CHEATED!"
            # 计算文本大小以居中显示（基于完整画面区域）
            text_size = cv2.getTextSize(warning_text, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)[0]
            text_x = (width - text_size[0]) // 2  # 在摄像头画面区域居中
            text_y = height // 2

            # 添加背景矩形使文字更醒目
            padding = 20
            cv2.rectangle(combined_display,
                         (text_x - padding, text_y - text_size[1] - padding),
                         (text_x + text_size[0] + padding, text_y + padding),
                         (0, 0, 0), -1)

            # 显示红色警告文字
            cv2.putText(combined_display, warning_text, (text_x, text_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3, cv2.LINE_AA)

        # FPS显示
        if now - last_time >= 1.0:
            fps = frame_count / (now - last_time)
            frame_count = 0
            last_time = now
        else:
            fps = 0

        if fps > 0:
            cv2.putText(combined_display, f"FPS: {fps:.1f}",
                        (total_width - 120, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow("RPS - Visual Optimisation (ROI Detection)", combined_display)

        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    try:
        if lock_events > 0:
            r90 = 100.0 * lock_vis_low_90 / lock_events
            r95 = 100.0 * lock_vis_low_95 / lock_events
            print(f"[SUMMARY] locks={lock_events} | <0.90={lock_vis_low_90} ({r90:.2f}%) | <0.95={lock_vis_low_95} ({r95:.2f}%)")
    except Exception as e:
        print(f"Summary failed: {e}")

    image_receiver.close()
    cv2.destroyAllWindows()
    try:
        _udp_sock.close()
        _udp_sock_arm.close()
    except Exception:
        pass
