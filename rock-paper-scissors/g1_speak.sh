#!/bin/bash
# G1 TTS 包装脚本 - 调用编译好的 g1_speak 节点
# 用法: ./g1_speak.sh "You cheated!"

TEXT="$1"

if [ -z "$TEXT" ]; then
    echo "Usage: $0 \"text to speak\""
    exit 1
fi

# 项目目录
PROJECT_DIR="$HOME/g1_rps_project"
WS_G1_DIR="$PROJECT_DIR/ws_G1"

# G1 ROS2 环境配置
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/setup.sh
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
source "$WS_G1_DIR/install/setup.bash"

# 调用编译好的 g1_speak 节点
# 该节点使用 AudioClient::TtsMaker() 通过 "voice" service 与 G1 通信
G1_SPEAK_BIN="$WS_G1_DIR/install/g1_tts/lib/g1_tts/g1_speak"

if [ -x "$G1_SPEAK_BIN" ]; then
    echo "[G1 TTS] Calling g1_speak: $TEXT"
    $G1_SPEAK_BIN "$TEXT"
    exit_code=$?
    if [ $exit_code -eq 0 ]; then
        echo "[G1 TTS] Success"
        exit 0
    else
        echo "[G1 TTS] g1_speak returned error code: $exit_code"
        exit $exit_code
    fi
else
    echo "[G1 TTS] ERROR: g1_speak binary not found at $G1_SPEAK_BIN"
    echo "[G1 TTS] Please build the g1_tts package:"
    echo "    cd $WS_G1_DIR && colcon build --packages-select g1_tts"
    exit 1
fi
