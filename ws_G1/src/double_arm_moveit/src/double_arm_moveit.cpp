/**
 * This example demonstrates how to use ROS2 to send low-level motor commands of
 *unitree g1 robot
 **/
#include "common/motor_crc_hg.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"
#include <rclcpp_action/rclcpp_action.hpp> // ROS 2 Action 库
#include <control_msgs/action/follow_joint_trajectory.hpp> // 轨迹Action类型
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <stdint.h>
#include <array>
#include <functional> // 添加functional头文件
#include <map> // 添加map头文件
#include <string> // 添加string头文件
// 使用placeholders命名空间
using std::placeholders::_1;
using std::placeholders::_2;

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

constexpr bool INFO_IMU = false;    // Set 1 to info IMU states
constexpr bool INFO_MOTOR = false;  // Set 1 to info motor states
constexpr bool HIGH_FREQ = true;
// Set 1 to subscribe to low states with high frequencies (500Hz)

enum PRorAB { PR = 0, AB = 1 };

constexpr int G1_NUM_MOTOR = 29;

enum G1JointIndex {
  LEFT_HIP_PITCH = 0,
  LEFT_HIP_ROLL = 1,
  LEFT_HIP_YAW = 2,
  LEFT_KNEE = 3,
  LEFT_ANKLE_PITCH = 4,
  LEFT_ANKLE_B = 4,
  LEFT_ANKLE_ROLL = 5,
  LEFT_ANKLE_A = 5,
  RIGHT_HIP_PITCH = 6,
  RIGHT_HIP_ROLL = 7,
  RIGHT_HIP_YAW = 8,
  RIGHT_KNEE = 9,
  RIGHT_ANKLE_PITCH = 10,
  RIGHT_ANKLE_B = 10,
  RIGHT_ANKLE_ROLL = 11,
  RIGHT_ANKLE_A = 11,
  WAIST_YAW = 12,
  WAIST_ROLL = 13,   // NOTE INVALID for g1 23dof/29dof with waist locked
  WAIST_A = 13,      // NOTE INVALID for g1 23dof/29dof with waist locked
  WAIST_PITCH = 14,  // NOTE INVALID for g1 23dof/29dof with waist locked
  WAIST_B = 14,      // NOTE INVALID for g1 23dof/29dof with waist locked
  LEFT_SHOULDER_PITCH = 15,
  LEFT_SHOULDER_ROLL = 16,
  LEFT_SHOULDER_YAW = 17,
  LEFT_ELBOW = 18,
  LEFT_WRIST_ROLL = 19,
  LEFT_WRIST_PITCH = 20,  // NOTE INVALID for g1 23dof
  LEFT_WRIST_YAW = 21,    // NOTE INVALID for g1 23dof
  RIGHT_SHOULDER_PITCH = 22,
  RIGHT_SHOULDER_ROLL = 23,
  RIGHT_SHOULDER_YAW = 24,
  RIGHT_ELBOW = 25,
  RIGHT_WRIST_ROLL = 26,
  RIGHT_WRIST_PITCH = 27,  // NOTE INVALID for g1 23dof
  RIGHT_WRIST_YAW = 28     // NOTE INVALID for g1 23dof
};
double time_scale_factor_ = 1.0;

// 创建结构体管理每个手臂的轨迹执行状态
struct ArmTrajectoryExecution {
  trajectory_msgs::msg::JointTrajectory trajectory;
  size_t current_point_index;
  double start_time;
  bool executing;
  std::shared_ptr<GoalHandle> goal_handle;
};

// Create a low_level_cmd_sender class for low state receive
class LowLevelCmdSender : public rclcpp::Node {
 public:
  LowLevelCmdSender() : Node("low_level_cmd_sender") {
    // 修改点1: 将auto改为const char*并初始化为"/lowstate"
    const char* topic_name = "/lowstate";  // 修改后的话题名称
    if (HIGH_FREQ) {
      topic_name = "lowstate";
    }

    // 修改点2: 使用BestEffort QoS策略以适应Isaac仿真
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    
    // The suber callback function is bind to
    // low_level_cmd_sender::topic_callback
    lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
        topic_name, qos_profile,  // 使用配置的QoS配置文件
        [this](const unitree_hg::msg::LowState::SharedPtr message) {
          LowStateHandler(message);
        });

    // the lowcmd_publisher_ is set to subscribe "/lowcmd" topic
    // 保持发布者使用Reliable QoS，确保控制命令可靠传输
    lowcmd_publisher_ =
        this->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);

    // The timer is set to 500 Hz, and bind to low_level_cmd_sender::Control
    // function
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_dt_),
                                     [this] { Control(); });

    // 初始化左右臂的轨迹执行状态
    left_arm_exec_.executing = false;
    right_arm_exec_.executing = false;

    // 创建SRDF关节名到G1电机索引的映射
    srdf_joint_to_motor_index_ = {
      {"right_shoulder_pitch_joint", RIGHT_SHOULDER_PITCH},
      {"right_shoulder_roll_joint", RIGHT_SHOULDER_ROLL},
      {"right_shoulder_yaw_joint", RIGHT_SHOULDER_YAW},
      {"right_elbow_joint", RIGHT_ELBOW},
      {"right_wrist_roll_joint", RIGHT_WRIST_ROLL},
      {"right_wrist_pitch_joint", RIGHT_WRIST_PITCH},
      {"right_wrist_yaw_joint", RIGHT_WRIST_YAW},
      
      {"left_shoulder_pitch_joint", LEFT_SHOULDER_PITCH},
      {"left_shoulder_roll_joint", LEFT_SHOULDER_ROLL},
      {"left_shoulder_yaw_joint", LEFT_SHOULDER_YAW},
      {"left_elbow_joint", LEFT_ELBOW},
      {"left_wrist_roll_joint", LEFT_WRIST_ROLL},
      {"left_wrist_pitch_joint", LEFT_WRIST_PITCH},
      {"left_wrist_yaw_joint", LEFT_WRIST_YAW},
      // 注意：left_hand_palm_joint 可能需要特殊处理或忽略
    };
      
    // 初始化左臂动作服务器
    left_arm_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "/left_arm_controller/follow_joint_trajectory",
      std::bind(&LowLevelCmdSender::handle_goal, this, _1, _2, "left"),
      std::bind(&LowLevelCmdSender::handle_cancel, this, _1),
      std::bind(&LowLevelCmdSender::handle_accepted, this, _1, "left"));
    
    // 初始化右臂动作服务器
    right_arm_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "/right_arm_controller/follow_joint_trajectory",
      std::bind(&LowLevelCmdSender::handle_goal, this, _1, _2, "right"),
      std::bind(&LowLevelCmdSender::handle_cancel, this, _1),
      std::bind(&LowLevelCmdSender::handle_accepted, this, _1, "right"));
      
    // Running time count
    time_ = 0;

    duration_ = 3;  // 3 s
    
    timer_dt_ = static_cast<int>(control_dt_ * 1000);

    RCLCPP_INFO(this->get_logger(), "LowLevelCmdSender 已初始化，等待轨迹目标...");

    // 新增：时间缩放因子（可根据需要调整或通过参数服务器动态配置）
    time_scale_factor_ = 1.0; // 示例：将所有轨迹点的执行时间延长一倍
    RCLCPP_INFO(this->get_logger(), "时间缩放因子设置为: %.1f", time_scale_factor_);
  }

 private:
  // Action服务器回调函数
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal,
    const std::string& arm_side)
  {
    RCLCPP_INFO(this->get_logger(), "收到%s臂的轨迹目标，包含 %zu 个路径点", 
               arm_side.c_str(), goal->trajectory.points.size());
    (void)uuid;

    // 简单验证：检查轨迹点是否为空
    if (goal->trajectory.points.empty()) {
      RCLCPP_ERROR(this->get_logger(), "接收到的轨迹点为空！");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    for (const auto& joint_name : goal->trajectory.joint_names) {
      if (srdf_joint_to_motor_index_.find(joint_name) == srdf_joint_to_motor_index_.end()) {
        RCLCPP_ERROR(this->get_logger(), "未知的关节名称: %s", joint_name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    RCLCPP_INFO(this->get_logger(), "%s臂目标验证通过，接受轨迹", arm_side.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
  {
    // 检查是左臂还是右臂的目标
    if (left_arm_exec_.executing && left_arm_exec_.goal_handle == goal_handle) {
      RCLCPP_INFO(this->get_logger(), "收到左臂取消请求");
      left_arm_exec_.executing = false;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    else if (right_arm_exec_.executing && right_arm_exec_.goal_handle == goal_handle) {
      RCLCPP_INFO(this->get_logger(), "收到右臂取消请求");
      right_arm_exec_.executing = false;
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    RCLCPP_WARN(this->get_logger(), "收到未知目标的取消请求");
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle,
                       const std::string& arm_side)
  {
    // 根据手臂侧选择对应的执行状态
    ArmTrajectoryExecution* arm_exec = nullptr;
    if (arm_side == "left") {
      arm_exec = &left_arm_exec_;
    } else if (arm_side == "right") {
      arm_exec = &right_arm_exec_;
    } else {
      RCLCPP_ERROR(this->get_logger(), "未知的手臂侧: %s", arm_side.c_str());
      return;
    }
    
    // 准备执行新轨迹
    arm_exec->trajectory = goal_handle->get_goal()->trajectory;
    arm_exec->current_point_index = 0;
    arm_exec->start_time = time_; // 使用全局时间time_作为参考
    arm_exec->executing = true;
    arm_exec->goal_handle = goal_handle;

    RCLCPP_INFO(this->get_logger(), "开始执行%s臂新轨迹", arm_side.c_str());
  }

  // 执行单个手臂的轨迹
  void execute_arm_trajectory(ArmTrajectoryExecution& arm_exec, const std::string& arm_side) {
    if (!arm_exec.executing) {
      return;
    }
    
    // 计算从轨迹开始实际经过的时间
    double elapsed_time = (time_ - arm_exec.start_time) * time_scale_factor_;
    
    // 检查是否已经执行完所有轨迹点
    if (arm_exec.current_point_index >= arm_exec.trajectory.points.size()) {
      // 获取最后一个点的时间
      double last_point_time = rclcpp::Duration(
        arm_exec.trajectory.points.back().time_from_start).seconds();
      
      // 等待最后一个点的时间结束
      if (elapsed_time >= last_point_time) {
        RCLCPP_INFO(this->get_logger(), "%s臂轨迹执行完成", arm_side.c_str());
        arm_exec.executing = false;
        if (arm_exec.goal_handle) {
          auto result = std::make_shared<FollowJointTrajectory::Result>();
          result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
          arm_exec.goal_handle->succeed(result);
          arm_exec.goal_handle.reset();
          RCLCPP_INFO(this->get_logger(), "已发送%s臂成功结果", arm_side.c_str());
        }
      }
      return;
    }
    
    // 获取当前轨迹点
    const auto& point = arm_exec.trajectory.points[arm_exec.current_point_index];
    double point_time = rclcpp::Duration(point.time_from_start).seconds();
    
    // 如果当前时间已经达到这个轨迹点的时间，则切换到下一个点
    if (elapsed_time >= point_time) {
      arm_exec.current_point_index++;
      RCLCPP_DEBUG(this->get_logger(), "%s臂切换到下一个轨迹点: %zu", 
                  arm_side.c_str(), arm_exec.current_point_index);
      
      // 检查是否到达轨迹终点
      if (arm_exec.current_point_index >= arm_exec.trajectory.points.size()) {
        RCLCPP_INFO(this->get_logger(), "%s臂已到达轨迹终点", arm_side.c_str());
        return;
      }
    }
    
    // 应用当前轨迹点到电机命令
    for (size_t i = 0; i < arm_exec.trajectory.joint_names.size(); i++) {
      const std::string& joint_name = arm_exec.trajectory.joint_names[i];
      auto it = srdf_joint_to_motor_index_.find(joint_name);
      
      if (it != srdf_joint_to_motor_index_.end() && i < point.positions.size()) {
        int motor_index = it->second;
        if (motor_index < G1_NUM_MOTOR) {
          // 获取目标位置和速度
          float target_position = point.positions[i];
          float target_velocity = (i < point.velocities.size()) ? point.velocities[i] : 0.0f;
          
          // 获取当前位置（从状态反馈）
          float current_position = motor_[motor_index].q;
          float current_velocity = motor_[motor_index].dq;
          
          // PD控制计算补偿力矩（减少抖动）
          float position_error = target_position - current_position;
          float velocity_error = target_velocity - current_velocity;
          float tau_compensation = 5.0f * position_error + 0.1f * velocity_error;
          
          RCLCPP_DEBUG(this->get_logger(), "%s臂关节 %s: 目标=%.3f, 当前=%.3f, 误差=%.3f", 
                      arm_side.c_str(), joint_name.c_str(), 
                      target_position, current_position, position_error);
          
          // 设置电机命令
          low_command_.motor_cmd[motor_index].q = target_position;
          low_command_.motor_cmd[motor_index].dq = target_velocity;
          low_command_.motor_cmd[motor_index].tau = tau_compensation;
          low_command_.motor_cmd[motor_index].kp = 80.0;
          low_command_.motor_cmd[motor_index].kd = 1.0;
        }
      }
    }
  }
 
  void Control() {
    // Test code here
    time_ += control_dt_;
    low_command_.mode_pr = mode_;
    low_command_.mode_machine = mode_machine_;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
     low_command_.motor_cmd[i].mode = 1;  // 1:Enable, 0:Disable
    }

    if (time_ < duration_) {
      // [Stage 1]: set robot to zero posture
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        double const ratio = clamp(time_ / duration_, 0.0, 1.0);
        low_command_.motor_cmd[i].q =
            static_cast<float>((1. - ratio) * motor_[i].q);
      }
    } else {
      // [Stage 2]: 轨迹跟踪与控制
      mode_ = PRorAB::PR;  // 启用PR模式
      
      // 执行左臂轨迹
      execute_arm_trajectory(left_arm_exec_, "左");
      
      // 执行右臂轨迹
      execute_arm_trajectory(right_arm_exec_, "右");
    }
    
    get_crc(low_command_);
    lowcmd_publisher_->publish(low_command_);
  }

  void LowStateHandler(const unitree_hg::msg::LowState::SharedPtr &message) {
    mode_machine_ = static_cast<int>(message->mode_machine);
    imu_ = message->imu_state;
    for (int i = 0; i < G1_NUM_MOTOR; i++) {
      motor_[i] = message->motor_state[i];
    }

    if (INFO_IMU) {
      // Info IMU states
      // RPY euler angle(ZYX order respected to body frame)
      // Quaternion
      // Gyroscope (raw data)
      // Accelerometer (raw data)
      RCLCPP_INFO(this->get_logger(),
                  "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu_.rpy[0],
                  imu_.rpy[1], imu_.rpy[2]);
      RCLCPP_INFO(this->get_logger(),
                  "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f",
                  imu_.quaternion[0], imu_.quaternion[1], imu_.quaternion[2],
                  imu_.quaternion[3]);
      RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f",
                  imu_.gyroscope[0], imu_.gyroscope[1], imu_.gyroscope[2]);
      RCLCPP_INFO(this->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f",
                  imu_.accelerometer[0], imu_.accelerometer[1],
                  imu_.accelerometer[2]);
    }
    if (INFO_MOTOR) {
      // Info motor states
      // q: angluar (rad)
      // dq: angluar velocity (rad/s)
      // ddq: angluar acceleration (rad/(s^2))
      // tau_est: Estimated external torque
      for (int i = 0; i < G1_NUM_MOTOR; i++) {
        motor_[i] = message->motor_state[i];
        RCLCPP_INFO(this->get_logger(),
                    "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
                    i, motor_[i].q, motor_[i].dq, motor_[i].ddq,
                    motor_[i].tau_est);
      }
    }
  }

  static double clamp(double value, double low, double high) {
    if (value < low) {
      return low;
    }
    if (value > high) {
      return high;
    }
    return value;
  }

  // 新增成员变量
  std::map<std::string, int> srdf_joint_to_motor_index_; // SRDF关节名到电机索引的映射

  // 左右臂的动作服务器
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr left_arm_action_server_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr right_arm_action_server_;
  
  // 左右臂的轨迹执行状态
  ArmTrajectoryExecution left_arm_exec_;
  ArmTrajectoryExecution right_arm_exec_;

  rclcpp::TimerBase::SharedPtr timer_;  // ROS2 timer
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr
      lowcmd_publisher_;  // ROS2 Publisher
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr
      lowstate_subscriber_;              // ROS2 Subscriber
  unitree_hg::msg::LowCmd low_command_;  // Unitree hg lowcmd message
  unitree_hg::msg::IMUState imu_;        // Unitree hg IMU message
  std::array<unitree_hg::msg::MotorState, G1_NUM_MOTOR>
      motor_;                  // Unitree hg motor state message
  double control_dt_ = 0.002;  // 2ms
  int timer_dt_ = static_cast<int>(control_dt_ * 1000);
  double time_;  // Running time count
  double duration_;
  PRorAB mode_ = PRorAB::PR;
  int mode_machine_{};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);  // Initialize rclcpp
  rclcpp::TimerBase::SharedPtr const
      timer_;  // Create a timer callback object to send cmd in time intervals
  auto node =
      std::make_shared<LowLevelCmdSender>();  // Create a ROS2 node and make
                                              // share with
                                              // low_level_cmd_sender class
  rclcpp::spin(node);                         // Run ROS2 node
  rclcpp::shutdown();                         // Exit
  return 0;
}
