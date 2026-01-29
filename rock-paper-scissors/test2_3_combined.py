#!/usr/bin/env python3
"""
Rock-Paper-Scissors 综合版 - 必胜模式 + 随机模式 一键切换
结合 test6_visual_optimisation.py 和 test2_3.py 的功能

模式切换：
- 按 'M' 键切换模式
- 随机模式：检测到手就随机出拳（简单快速）
- 必胜模式：使用启发式规则+Markov预测来智能选择AI出拳

通过 TCP 接收 realsense_image_bridge.py 发送的彩色图
与 rps_game.launch.py 配合使用：
- 通过 UDP:5005 发送 AI 手势命令
- 通过 UDP:5007 发送手臂控制命令
"""

import cv2
import mediapipe as mp
import numpy as np
import time
import os
import socket
import struct
import select
import random
import subprocess
import threading
from collections import deque

# 获取项目根目录
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)

# -------------------------
# TCP 连接配置 (接收图像)
# -------------------------
TCP_IP = "127.0.0.1"
TCP_PORT = 5006

# -------------------------
# UDP 发送配置 (控制机器人)
# -------------------------
UDP_IP = "127.0.0.1"
UDP_PORT_MOVE = 5005      # AI 手势命令端口
UDP_PORT_ARM = 5007       # 手臂控制命令端口

_udp_sock_move = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_udp_sock_arm = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_ai_move(ai_move_name: str):
    """发送 AI 手势命令: 'rock', 'paper', 'scissors'"""
    msg = ai_move_name.strip().lower().encode("utf-8")
    _udp_sock_move.sendto(msg, (UDP_IP, UDP_PORT_MOVE))
    print(f"[UDP] Sent AI move -> {ai_move_name}")


def send_arm_command(cmd: str):
    """发送手臂控制命令: 'raise' 或 'lower'"""
    msg = cmd.strip().lower().encode("utf-8")
    _udp_sock_arm.sendto(msg, (UDP_IP, UDP_PORT_ARM))
    print(f"[ARM] Sent arm command -> {cmd}")


# -------------------------
# TTS (Text-to-Speech) Function
# -------------------------
G1_SPEAK_PATH = os.path.join(PROJECT_ROOT, "g1_speak.sh")

def speak_async(text):
    """异步播放TTS，不阻塞主线程"""
    def _speak():
        try:
            if os.path.exists(G1_SPEAK_PATH):
                result = subprocess.run([G1_SPEAK_PATH, text],
                                      capture_output=True, timeout=5)
                if result.returncode == 0:
                    return
            subprocess.run(['espeak', text, '-s', '150', '-v', 'en+f3'],
                          check=True, capture_output=True, timeout=3)
        except Exception as e:
            print(f"[TTS] Failed: {e}")

    thread = threading.Thread(target=_speak, daemon=True)
    thread.start()


# -------------------------
# MediaPipe Hands 初始化
# -------------------------
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.4,
    min_tracking_confidence=0.4,
)

# -------------------------
# 检测区域限制参数 (ROI)
# -------------------------
DETECTION_ROI_ENABLED = True
DETECTION_ROI_WIDTH_RATIO = 0.4
DETECTION_ROI_START_RATIO = 0.4

# -------------------------
# 电脑出拳的图片
# -------------------------
ai_images = {
    "rock": cv2.imread(os.path.join(PROJECT_ROOT, "images", "rock.png")),
    "paper": cv2.imread(os.path.join(PROJECT_ROOT, "images", "paper.png")),
    "scissors": cv2.imread(os.path.join(PROJECT_ROOT, "images", "scissors.png")),
}

# -------------------------
# 模式配置
# -------------------------
MODE_RANDOM = "RANDOM"      # 随机模式
MODE_SMART = "SMART"        # 必胜模式
current_mode = MODE_SMART   # 默认必胜模式

# -------------------------
# 随机出拳
# -------------------------
AI_MOVES = ["rock", "scissors", "paper"]

# -------------------------
# 通用状态
# -------------------------
ai_move_name = None
user_move_name = None
LOCK_HOLD_SEC = 2.0
locked_until = 0.0

# 手臂控制
arm_raised = False
ARM_RAISE_DELAY = 0.8
arm_raise_scheduled_at = 0.0

# -------------------------
# 必胜模式专用：Markov模型和手势识别
# -------------------------
REV_CLASS_MAP = {0: "paper", 1: "rock", 2: "scissors"}
MOVE_TO_IDX = {"paper": 0, "rock": 1, "scissors": 2}
IDX_TO_MOVE = {v: k for k, v in MOVE_TO_IDX.items()}

trans_counts = np.ones((3, 3), dtype=np.float32)
base_counts = np.ones(3, dtype=np.float32)
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

# 运动检测参数
MOVE_START_TH = 0.006
MOVE_END_TH = 0.005
END_STILL_FRAMES = 1
MOVE_HISTORY = 2
MIN_DECISION_FRAMES = 3
USE_DECELERATION = True
DECEL_RATIO = 0.6

# 作弊检测
CHEAT_DETECTION_ENABLED = True
CHEAT_CONFIDENCE_THRESHOLD = 0.70
CHEAT_WARNING_DURATION = 1.5

# 必胜模式状态
move_q = deque(maxlen=MOVE_HISTORY)
decision_q = deque(maxlen=50)
in_motion = False
still_count = 0
prev_wrist = None
peak_move = 0.0
locked_gesture = None
cheat_detected = False
cheat_warning_until = 0.0

# 调试信息
dbg_vis_name = None
dbg_vis_conf = None
dbg_heur_name = None
dbg_heur_conf = None


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


def mapper(val: int) -> str:
    return REV_CLASS_MAP.get(val, "unknown")


def heuristic_gesture_check(landmarks):
    """基于几何特征判断手势类型"""
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

    def get_point(idx):
        return np.array([landmarks[idx].x, landmarks[idx].y, landmarks[idx].z])

    wrist = get_point(WRIST)

    def is_extended(tip_idx, pip_idx):
        tip = get_point(tip_idx)
        pip = get_point(pip_idx)
        dist_tip = np.linalg.norm(tip - wrist)
        dist_pip = np.linalg.norm(pip - wrist)
        return dist_tip > dist_pip * 1.1

    thumb_ext = is_extended(THUMB_TIP, 2)
    index_ext = is_extended(INDEX_TIP, INDEX_PIP)
    middle_ext = is_extended(MIDDLE_TIP, MIDDLE_PIP)
    ring_ext = is_extended(RING_TIP, RING_PIP)
    pinky_ext = is_extended(PINKY_TIP, PINKY_PIP)

    extended_count = sum([thumb_ext, index_ext, middle_ext, ring_ext, pinky_ext])

    if extended_count == 0 or extended_count == 1:
        return "rock", 0.85

    if extended_count >= 4:
        return "paper", 0.85

    if index_ext and middle_ext and not ring_ext and not pinky_ext:
        return "scissors", 0.90

    if index_ext and middle_ext:
        return "scissors", 0.75

    if extended_count == 2:
        if index_ext or middle_ext:
            return "scissors", 0.65

    return None, 0.0


# -------------------------
# 网络通信函数
# -------------------------
def recv_exact(sock, n):
    """精确接收 n 字节"""
    data = b''
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("Connection closed")
        data += chunk
    return data


def connect_to_bridge():
    """连接到 RealSense bridge"""
    print(f"Connecting to {TCP_IP}:{TCP_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131072)
    sock.connect((TCP_IP, TCP_PORT))
    print("Connected!")
    return sock


def receive_frame(sock):
    """接收一帧彩色图像"""
    try:
        ts_data = recv_exact(sock, 8)
        timestamp = struct.unpack('>d', ts_data)[0]

        size_data = recv_exact(sock, 4)
        size = struct.unpack('>I', size_data)[0]

        img_data = recv_exact(sock, size)
        color_frame = cv2.imdecode(np.frombuffer(img_data, np.uint8), cv2.IMREAD_COLOR)

        if color_frame is None:
            return None, None

        return color_frame, timestamp

    except Exception as e:
        print(f"[WARN] Receive error: {e}")
        return None, None


def has_pending_data(sock, timeout=0):
    """检查 socket 是否有待读取的数据"""
    readable, _, _ = select.select([sock], [], [], timeout)
    return len(readable) > 0


def get_latest_frame(sock):
    """获取最新帧，丢弃旧帧"""
    latest_frame = receive_frame(sock)

    while has_pending_data(sock, timeout=0):
        new_frame = receive_frame(sock)
        if new_frame[0] is not None:
            latest_frame = new_frame

    return latest_frame


def reset_smart_mode_state():
    """重置必胜模式的状态"""
    global move_q, decision_q, in_motion, still_count, prev_wrist, peak_move
    global locked_gesture, cheat_detected, cheat_warning_until

    move_q.clear()
    decision_q.clear()
    in_motion = False
    still_count = 0
    prev_wrist = None
    peak_move = 0.0
    locked_gesture = None
    cheat_detected = False
    cheat_warning_until = 0.0


# -------------------------
# 主程序
# -------------------------
def main():
    global current_mode
    global ai_move_name, user_move_name, locked_until
    global arm_raised, arm_raise_scheduled_at
    global trans_counts, base_counts, prev_locked_move
    global move_q, decision_q, in_motion, still_count, prev_wrist, peak_move
    global locked_gesture, cheat_detected, cheat_warning_until
    global dbg_vis_name, dbg_vis_conf, dbg_heur_name, dbg_heur_conf

    sock = connect_to_bridge()

    print("=" * 60)
    print("Rock-Paper-Scissors Combined Version")
    print("=" * 60)
    print(f"Current Mode: {current_mode}")
    print("Press 'M' to switch between RANDOM and SMART mode")
    print("Press ESC to quit")
    print("=" * 60)

    # 启动时抬起手臂
    send_arm_command("raise")
    arm_raised = True

    skip_count = 0
    fps_start_time = time.time()
    fps_frame_count = 0
    current_fps = 0

    while True:
        color_frame, timestamp = get_latest_frame(sock)

        if color_frame is None:
            skip_count += 1
            if skip_count > 50:
                print("Too many failed frames, reconnecting...")
                sock.close()
                time.sleep(1)
                try:
                    sock = connect_to_bridge()
                    skip_count = 0
                except Exception as e:
                    print(f"Reconnect failed: {e}")
                    break
            continue

        skip_count = 0
        fps_frame_count += 1
        now = time.time()

        # 检查是否需要抬起手臂
        if arm_raise_scheduled_at > 0 and now >= arm_raise_scheduled_at:
            if not arm_raised:
                send_arm_command("raise")
                arm_raised = True
            arm_raise_scheduled_at = 0.0

        # 计算帧率
        fps_elapsed = now - fps_start_time
        if fps_elapsed >= 1.0:
            current_fps = fps_frame_count / fps_elapsed
            fps_frame_count = 0
            fps_start_time = now

        # 镜像翻转
        color_frame = cv2.flip(color_frame, 1)
        height, width = color_frame.shape[:2]

        # -------------------------
        # 检测区域限制 (ROI)
        # -------------------------
        if DETECTION_ROI_ENABLED:
            roi_width = int(width * DETECTION_ROI_WIDTH_RATIO)
            roi_x_start = int(width * DETECTION_ROI_START_RATIO)
            roi_x_end = min(roi_x_start + roi_width, width)

            roi = color_frame[:, roi_x_start:roi_x_end]
            image_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
            results = hands.process(image_rgb)

            cv2.rectangle(color_frame, (roi_x_start, 0), (roi_x_end - 1, height - 1), (0, 255, 0), 2)
        else:
            roi_x_start = 0
            roi_width = width
            image_rgb = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
            results = hands.process(image_rgb)

        locked = now < locked_until
        has_hand = False
        status_msg = "Waiting for hand..."

        # -------------------------
        # 手部检测和处理
        # -------------------------
        if results.multi_hand_landmarks:
            has_hand = True
            hand_landmarks = results.multi_hand_landmarks[0]
            landmarks = hand_landmarks.landmark

            # 可视化：画手部关键点
            for landmark in landmarks:
                if DETECTION_ROI_ENABLED:
                    x = int(roi_x_start + landmark.x * roi_width)
                else:
                    x = int(landmark.x * width)
                y = int(landmark.y * height)
                cv2.circle(color_frame, (x, y), 3, (0, 255, 0), -1)

            # =====================
            # 随机模式逻辑
            # =====================
            if current_mode == MODE_RANDOM:
                if not locked:
                    ai_move_name = random.choice(AI_MOVES)
                    locked_until = now + LOCK_HOLD_SEC
                    print(f"[RANDOM] Hand detected, AI: {ai_move_name}")

                    send_ai_move(ai_move_name)

                    if arm_raised:
                        send_arm_command("lower")
                        arm_raised = False
                        arm_raise_scheduled_at = now + ARM_RAISE_DELAY

                    status_msg = f"AI: {ai_move_name}"
                else:
                    status_msg = f"Locked - AI: {ai_move_name}"

            # =====================
            # 必胜模式逻辑
            # =====================
            elif current_mode == MODE_SMART:
                # 运动检测
                if DETECTION_ROI_ENABLED:
                    wrist_x_in_full = (roi_x_start + landmarks[0].x * roi_width) / width
                    wrist_xy = (wrist_x_in_full, landmarks[0].y)
                else:
                    wrist_xy = (landmarks[0].x, landmarks[0].y)

                mean_move = None
                if prev_wrist is not None:
                    move = np.linalg.norm(np.array(wrist_xy) - np.array(prev_wrist))
                    move_q.append(move)
                    mean_move = float(np.mean(move_q)) if len(move_q) else move
                prev_wrist = wrist_xy

                # 启发式手势检测
                heur_gesture, heur_conf = heuristic_gesture_check(landmarks)
                dbg_heur_name = heur_gesture
                dbg_heur_conf = heur_conf

                # 生成视觉概率分布
                if heur_gesture is not None and heur_conf > 0.5:
                    p_vis = np.array([0.05, 0.05, 0.05], dtype=np.float32)
                    heur_idx = MOVE_TO_IDX[heur_gesture]
                    p_vis[heur_idx] = heur_conf
                    p_vis = p_vis / p_vis.sum()
                else:
                    p_vis = np.array([1.0/3, 1.0/3, 1.0/3], dtype=np.float32)

                vis_top = int(np.argmax(p_vis))
                live_vis_name = mapper(vis_top)
                live_vis_conf = float(p_vis[vis_top])
                dbg_vis_name = live_vis_name
                dbg_vis_conf = live_vis_conf

                # 预测分布和融合
                p_pred = _pred_dist_mixed(prev_locked_move)
                alpha = float(np.clip(live_vis_conf, FUSE_ALPHA_MIN, FUSE_ALPHA_MAX))
                p_final = _fuse_probs(p_vis, p_pred, alpha=alpha)

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
                                print(f"[SMART] User: {user_move_name}, AI: {ai_move_name}")

                                send_ai_move(ai_move_name)
                                speak_async("我赢了")
                                locked_until = now + LOCK_HOLD_SEC

                                if arm_raised:
                                    send_arm_command("lower")
                                    arm_raised = False
                                    arm_raise_scheduled_at = now + ARM_RAISE_DELAY

                                locked_gesture = user_move_name
                                cheat_detected = False

                                in_motion = False
                                still_count = 0
                                peak_move = 0.0
                                decision_q.clear()
                                move_q.clear()
                                status_msg = "Locked"
                else:
                    # 锁定期间的作弊检测
                    status_msg = "Locked"
                    if CHEAT_DETECTION_ENABLED and locked_gesture is not None:
                        if heur_gesture is not None and heur_conf > CHEAT_CONFIDENCE_THRESHOLD:
                            if heur_gesture != locked_gesture:
                                if not cheat_detected:
                                    cheat_detected = True
                                    cheat_warning_until = now + CHEAT_WARNING_DURATION
                                    print(f"[CHEAT] Gesture changed from {locked_gesture} to {heur_gesture}!")
                                    # speak_async("You cheated!")  # 暂时注释掉
                                status_msg = "CHEAT DETECTED!"

        else:
            # 没有检测到手
            prev_wrist = None
            if not locked:
                if current_mode == MODE_SMART:
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

        # -------------------------
        # 显示画面 - 全画面 + AI面板
        # -------------------------
        ai_panel_width = width // 3
        if ai_move_name and ai_move_name in ai_images and ai_images[ai_move_name] is not None:
            ai_move_display = cv2.resize(ai_images[ai_move_name], (ai_panel_width, height))
        else:
            ai_move_display = np.zeros((height, ai_panel_width, 3), dtype=np.uint8)

        combined_display = np.hstack((color_frame, ai_move_display))
        total_width = width + ai_panel_width

        # 模式指示器（左上角）
        mode_color = (0, 255, 255) if current_mode == MODE_RANDOM else (255, 0, 255)
        mode_text = f"Mode: {current_mode} (Press M to switch)"
        cv2.putText(combined_display, mode_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2, cv2.LINE_AA)

        # 必胜模式额外信息
        if current_mode == MODE_SMART:
            y_offset = 60
            if dbg_heur_name and dbg_heur_conf:
                cv2.putText(combined_display, f"Gesture: {dbg_heur_name} ({dbg_heur_conf:.2f})",
                            (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 150, 0), 2, cv2.LINE_AA)
                y_offset += 25
            if dbg_vis_name and dbg_vis_conf:
                cv2.putText(combined_display, f"Visual: {dbg_vis_name} ({dbg_vis_conf:.2f})",
                            (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)

        # 状态信息（左下角）
        cv2.putText(combined_display, status_msg, (10, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0) if has_hand else (255, 255, 255), 2, cv2.LINE_AA)

        # AI面板信息
        if ai_move_name:
            cv2.putText(combined_display, f"AI: {ai_move_name}",
                        (width + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
        if current_mode == MODE_SMART and user_move_name:
            cv2.putText(combined_display, f"User: {user_move_name}",
                        (width + 10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        # FPS显示
        if current_fps > 0:
            cv2.putText(combined_display, f"FPS: {current_fps:.1f}",
                        (total_width - 120, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

        # 作弊警告
        if current_mode == MODE_SMART and now < cheat_warning_until:
            warning_text = "YOU CHEATED!"
            text_size = cv2.getTextSize(warning_text, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)[0]
            text_x = (width - text_size[0]) // 2
            text_y = height // 2

            padding = 20
            cv2.rectangle(combined_display,
                         (text_x - padding, text_y - text_size[1] - padding),
                         (text_x + text_size[0] + padding, text_y + padding),
                         (0, 0, 0), -1)
            cv2.putText(combined_display, warning_text, (text_x, text_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3, cv2.LINE_AA)

        cv2.imshow("Rock-Paper-Scissors (Combined)", combined_display)

        # 按键处理
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break
        elif key == ord('m') or key == ord('M'):
            # 切换模式
            if current_mode == MODE_RANDOM:
                current_mode = MODE_SMART
                reset_smart_mode_state()
            else:
                current_mode = MODE_RANDOM

            # 重置锁定状态
            locked_until = 0.0
            ai_move_name = None
            user_move_name = None

            print(f"[MODE] Switched to {current_mode} mode")

    sock.close()
    cv2.destroyAllWindows()

    try:
        _udp_sock_move.close()
        _udp_sock_arm.close()
    except Exception:
        pass


if __name__ == "__main__":
    main()
