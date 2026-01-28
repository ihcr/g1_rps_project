# G1 Rock-Paper-Scissors Project

基于 Unitree G1 机器人的石头剪刀布游戏系统。

## 系统要求

- Ubuntu 22.04
- ROS2 Humble
- Conda (Miniconda/Miniforge)
- Intel RealSense 相机

## 项目结构

```
g1_rps_project/
├── README.md                        # 部署文档
├── bridge_dual_arm.py               # 双臂关节状态桥接脚本
│
├── rock-paper-scissors/             # 手势识别模块
│   ├── test6_visual_optimisation.py # 主程序（MediaPipe手势识别）
│   ├── realsense_image_bridge.py    # RealSense相机图像桥接
│   ├── ros_bridge.py                # UDP ↔ ROS2 话题桥接
│   └── requirements.txt             # Python依赖
│
└── ws_G1/src/                       # ROS2 工作空间
    ├── g1_robot_base.launch.py      # 启动文件1：机器人底层控制
    ├── rps_game.launch.py           # 启动文件2：游戏应用层
    ├── rps_arm_controller.py        # 手臂摆动控制
    │
    ├── g1arm_moveit/                # G1手臂控制节点
    ├── left_moveit/                 # 左臂MoveIt（含夹爪控制）
    ├── right_moveit/                # 右臂MoveIt
    ├── double_arm_moveit/           # 双臂MoveIt
    │
    ├── g1_arm_description/          # G1手臂URDF模型
    ├── left_arm_description/        # 左臂URDF
    ├── right_arm_description/       # 右臂URDF
    ├── DEX3-1/                      # 灵巧手模型(STL mesh)
    │
    ├── left_arm_moveit_config/      # 左臂MoveIt配置
    ├── right_arm_moveit_config/     # 右臂MoveIt配置
    ├── double_arm_moveit_config/    # 双臂MoveIt配置
    │
    └── g1_tts/                      # 语音合成模块
```

## 部署步骤

### 1. 安装系统依赖

```bash
# MoveIt2
sudo apt install ros-humble-moveit

# ros2-control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# CycloneDDS
sudo apt install ros-humble-rmw-cyclonedds-cpp

# RealSense (如果需要)
sudo apt install ros-humble-librealsense2* ros-humble-realsense2-*
```

### 2. 安装 Unitree ROS2 SDK

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_ros2.git
cd unitree_ros2
# 按照 unitree_ros2 官方文档编译 cyclonedds_ws 和 unitree_ros2_ws
```

### 3. 克隆本项目

```bash
cd ~
git clone https://github.com/ihcr/<仓库名>.git g1_rps_project
```

### 4. 编译 ROS2 工作空间

```bash
cd ~/g1_rps_project/ws_G1
colcon build
source install/setup.bash
```

### 5. 创建 Conda 环境

```bash
conda create -n rock_paper python=3.10
conda activate rock_paper
pip install -r ~/g1_rps_project/rock-paper-scissors/requirements.txt
```

## 运行方法

### 终端 1：启动机器人底层控制

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/setup.sh
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/g1_rps_project/ws_G1/install/setup.bash

ros2 launch ~/g1_rps_project/ws_G1/src/g1_robot_base.launch.py
```

### 终端 2：启动游戏应用层

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/setup.sh
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/g1_rps_project/ws_G1/install/setup.bash

ros2 launch ~/g1_rps_project/ws_G1/src/rps_game.launch.py
```

### 终端 3：启动手势识别（需要 Conda）

```bash
cd ~/g1_rps_project/rock-paper-scissors
conda activate rock_paper
python3 test6_visual_optimisation.py
```

## 依赖说明

### 仓库内包含
- ROS2 包（MoveIt配置、机器人描述、控制节点）
- Launch 文件
- 手势识别代码（基于 MediaPipe）

### 需单独安装
| 依赖 | 安装方式 |
|------|----------|
| ROS2 Humble | 系统预装 |
| MoveIt2 | `sudo apt install ros-humble-moveit` |
| ros2-control | `sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers` |
| CycloneDDS | `sudo apt install ros-humble-rmw-cyclonedds-cpp` |
| unitree_ros2 SDK | 单独 git clone 并编译 |
| Conda 环境 | 按步骤5创建 |

### Python 依赖 (Conda 环境)
- mediapipe==0.10.14
- opencv-python==4.10.0.84
- tensorflow==2.16.1
- numpy
- matplotlib
