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


// 初始化FollowJointTrajectory Action服务器 - 左臂
    action_server_left_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "/left_arm_controller/follow_joint_trajectory", // Action名称，与MoveIt配置匹配
     std::bind(&LowLevelCmdSender::handle_goal_left, this, _1, _2),
      std::bind(&LowLevelCmdSender::handle_cancel_left, this, _1),
      std::bind(&LowLevelCmdSender::handle_accepted_left, this, _1));

    // 初始化FollowJointTrajectory Action服务器 - 右臂
    action_server_right_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "/right_arm_controller/follow_joint_trajectory", // Action名称，与MoveIt配置匹配
     std::bind(&LowLevelCmdSender::handle_goal_right, this, _1, _2),
      std::bind(&LowLevelCmdSender::handle_cancel_right, this, _1),
      std::bind(&LowLevelCmdSender::handle_accepted_right, this, _1));
      
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
      
    // Running time count
    time_ = 0;

    duration_ = 3;  // 3 s
    
    timer_dt_ = static_cast<int>(control_dt_ * 1000);

    // 轨迹执行相关状态变量 - 左臂
    executing_trajectory_left_ = false;
    current_trajectory_point_index_left_ = 0;
    trajectory_start_time_left_ = 0.0;

    // 轨迹执行相关状态变量 - 右臂
    executing_trajectory_right_ = false;
    current_trajectory_point_index_right_ = 0;
    trajectory_start_time_right_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "LowLevelCmdSender 已初始化，等待左臂和右臂轨迹目标...");

    // 新增：时间缩放因子（可根据需要调整或通过参数服务器动态配置）
    time_scale_factor_ = 2.0; // 示例：将所有轨迹点的执行时间延长一倍
    RCLCPP_INFO(this->get_logger(), "时间缩放因子设置为: %.1f", time_scale_factor_);


  }

 private:

 // Action服务器回调函数 - 左臂
  rclcpp_action::GoalResponse handle_goal_left(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "[左臂] 收到新的轨迹目标，包含 %zu 个路径点", goal->trajectory.points.size());
    (void)uuid;

    // 简单验证：检查轨迹点是否为空
    if (goal->trajectory.points.empty()) {
      RCLCPP_ERROR(this->get_logger(), "[左臂] 接收到的轨迹点为空！");
      return rclcpp_action::GoalResponse::REJECT;
    }

    for (const auto& joint_name : goal->trajectory.joint_names) {
      if (srdf_joint_to_motor_index_.find(joint_name) == srdf_joint_to_motor_index_.end()) {
        RCLCPP_ERROR(this->get_logger(), "[左臂] 未知的关节名称: %s", joint_name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    RCLCPP_INFO(this->get_logger(), "[左臂] 目标验证通过，接受轨迹");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_left(
    const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "[左臂] 收到取消请求");
    executing_trajectory_left_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_left(const std::shared_ptr<GoalHandle> goal_handle)
  {
    goal_handle_left_ = goal_handle;
    current_trajectory_left_ = goal_handle->get_goal()->trajectory;
    current_trajectory_point_index_left_ = 0;
    trajectory_start_time_left_ = time_;
    executing_trajectory_left_ = true;
    RCLCPP_INFO(this->get_logger(), "[左臂] 开始执行新轨迹");
  }

 // Action服务器回调函数 - 右臂
  rclcpp_action::GoalResponse handle_goal_right(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "[右臂] 收到新的轨迹目标，包含 %zu 个路径点", goal->trajectory.points.size());
    (void)uuid;

    if (goal->trajectory.points.empty()) {
      RCLCPP_ERROR(this->get_logger(), "[右臂] 接收到的轨迹点为空！");
      return rclcpp_action::GoalResponse::REJECT;
    }

    for (const auto& joint_name : goal->trajectory.joint_names) {
      if (srdf_joint_to_motor_index_.find(joint_name) == srdf_joint_to_motor_index_.end()) {
        RCLCPP_ERROR(this->get_logger(), "[右臂] 未知的关节名称: %s", joint_name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }

    RCLCPP_INFO(this->get_logger(), "[右臂] 目标验证通过，接受轨迹");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_right(
    const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "[右臂] 收到取消请求");
    executing_trajectory_right_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_right(const std::shared_ptr<GoalHandle> goal_handle)
  {
    goal_handle_right_ = goal_handle;
    current_trajectory_right_ = goal_handle->get_goal()->trajectory;
    current_trajectory_point_index_right_ = 0;
    trajectory_start_time_right_ = time_;
    executing_trajectory_right_ = true;
    RCLCPP_INFO(this->get_logger(), "[右臂] 开始执行新轨迹");
  }

 
 
  void Control() {
    // Test code here
    time_ += control_dt_;
    low_command_.mode_pr = mode_;
    low_command_.mode_machine = mode_machine_;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
     low_command_.motor_cmd[i].mode = 1;  // 1:Enable, 0:Disable
    //   low_command_.motor_cmd[i].tau = 0.0;
    //   low_command_.motor_cmd[i].q = 0.0;
    //   low_command_.motor_cmd[i].dq = 0.0;
    //   low_command_.motor_cmd[i].kp = (i < 13) ? 100.0 : 50.0;
    //   low_command_.motor_cmd[i].kd = 1.0;
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

      // 处理左臂轨迹
      if (executing_trajectory_left_) {
        execute_trajectory(executing_trajectory_left_, current_trajectory_left_,
                          current_trajectory_point_index_left_, trajectory_start_time_left_,
                          goal_handle_left_, "[左臂]");
      }

      // 处理右臂轨迹
      if (executing_trajectory_right_) {
        execute_trajectory(executing_trajectory_right_, current_trajectory_right_,
                          current_trajectory_point_index_right_, trajectory_start_time_right_,
                          goal_handle_right_, "[右臂]");
      }
    }

    get_crc(low_command_);
    lowcmd_publisher_->publish(low_command_);
  }

  // 轨迹执行辅助函数
  void execute_trajectory(bool& executing, trajectory_msgs::msg::JointTrajectory& trajectory,
                         size_t& point_index, double& start_time,
                         std::shared_ptr<GoalHandle>& goal_handle, const std::string& arm_label) {
        RCLCPP_DEBUG(this->get_logger(), "%s 有活动轨迹，当前点索引: %zu", arm_label.c_str(), point_index);

        // 计算从轨迹开始实际经过的时间
        double elapsed_time = (time_ - start_time) * time_scale_factor_;
        RCLCPP_DEBUG(this->get_logger(), "%s 已过时间(缩放后): %.3f秒 (缩放因子: %.1f)", arm_label.c_str(), elapsed_time, time_scale_factor_);

        if (point_index < trajectory.points.size()) {
          const auto& point = trajectory.points[point_index];

          // ===== 保留所有打印语句开始 =====
          RCLCPP_INFO(this->get_logger(), "%s 执行轨迹点 %zu:", arm_label.c_str(), point_index);
          RCLCPP_INFO(this->get_logger(), "%s   time_from_start: %.3f 秒", arm_label.c_str(),
                     rclcpp::Duration(point.time_from_start).seconds());

          if (!point.positions.empty()) {
              std::string positions_str = arm_label + "   positions: ";
              for (size_t i = 0; i < point.positions.size(); ++i) {
                  positions_str += std::to_string(point.positions[i]) + " ";
              }
              RCLCPP_INFO(this->get_logger(), "%s", positions_str.c_str());

              // 同时打印对应的关节名称（如果可用）
              if (!trajectory.joint_names.empty() &&
                  point.positions.size() == trajectory.joint_names.size()) {
                  std::string joints_str = arm_label + "   关节对应值: ";
                  for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
                      joints_str += trajectory.joint_names[i] + "=" +
                                  std::to_string(point.positions[i]) + " ";
                  }
                  RCLCPP_INFO(this->get_logger(), "%s", joints_str.c_str());
              }
          } else {
              RCLCPP_WARN(this->get_logger(), "%s 轨迹点 %zu 的positions数组为空!", arm_label.c_str(), point_index);
          }
          // ===== 保留所有打印语句结束 =====

          // 基于时间推进选择轨迹点（而不是简单递增索引）
          double point_time = rclcpp::Duration(point.time_from_start).seconds();

          // 检查是否需要切换到下一个点
          size_t next_point_index = point_index;
          if (elapsed_time >= point_time && point_index + 1 < trajectory.points.size()) {
            next_point_index = point_index + 1;
            RCLCPP_DEBUG(this->get_logger(), "%s 准备插值到下一个轨迹点: %zu", arm_label.c_str(), next_point_index);
          }

          // 检查轨迹是否完成
          if (point_index >= trajectory.points.size() - 1) {
            double last_point_time = rclcpp::Duration(
              trajectory.points.back().time_from_start).seconds();

            if (elapsed_time >= last_point_time) {
              RCLCPP_INFO(this->get_logger(), "%s 轨迹执行完成", arm_label.c_str());
              executing = false;
              if (goal_handle) {
                auto result = std::make_shared<FollowJointTrajectory::Result>();
                result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
                goal_handle->succeed(result);
                goal_handle.reset();
                RCLCPP_INFO(this->get_logger(), "%s 已发送成功结果", arm_label.c_str());
              }
              return;
            }
          }

          // 计算插值后的目标位置（在当前点和下一个点之间线性插值）
          std::vector<float> interpolated_positions;
          std::vector<float> interpolated_velocities;

          if (next_point_index > point_index && next_point_index < trajectory.points.size()) {
            // 两点之间插值
            const auto& current_point = trajectory.points[point_index];
            const auto& next_point = trajectory.points[next_point_index];

            double t0 = rclcpp::Duration(current_point.time_from_start).seconds();
            double t1 = rclcpp::Duration(next_point.time_from_start).seconds();
            double alpha = (elapsed_time - t0) / (t1 - t0);
            alpha = std::max(0.0, std::min(1.0, alpha)); // 限制在 [0, 1]

            RCLCPP_DEBUG(this->get_logger(), "%s 插值系数 alpha=%.3f (t0=%.3f, t=%.3f, t1=%.3f)",
                        arm_label.c_str(), alpha, t0, elapsed_time, t1);

            for (size_t i = 0; i < current_point.positions.size(); i++) {
              float p0 = current_point.positions[i];
              float p1 = next_point.positions[i];
              float interpolated = p0 + alpha * (p1 - p0);
              interpolated_positions.push_back(interpolated);

              // 速度插值（如果有）
              if (i < current_point.velocities.size() && i < next_point.velocities.size()) {
                float v0 = current_point.velocities[i];
                float v1 = next_point.velocities[i];
                interpolated_velocities.push_back(v0 + alpha * (v1 - v0));
              } else {
                // 估算速度：(p1 - p0) / (t1 - t0)
                interpolated_velocities.push_back((p1 - p0) / (t1 - t0));
              }
            }

            // 切换到下一个段
            if (elapsed_time >= t1) {
              point_index = next_point_index;
              RCLCPP_DEBUG(this->get_logger(), "%s 切换到轨迹点: %zu", arm_label.c_str(), point_index);
            }
          } else {
            // 使用当前点的位置（没有下一个点可插值）
            interpolated_positions.assign(point.positions.begin(), point.positions.end());
            interpolated_velocities.assign(point.velocities.begin(), point.velocities.end());
          }

          // 将插值后的轨迹位置应用到相应的电机命令（PD闭环控制）
          for (size_t i = 0; i < trajectory.joint_names.size(); i++) {
            const std::string& joint_name = trajectory.joint_names[i];
            auto it = srdf_joint_to_motor_index_.find(joint_name);

            if (it != srdf_joint_to_motor_index_.end() && i < interpolated_positions.size()) {
              int motor_index = it->second;
              if (motor_index < G1_NUM_MOTOR) {
                // 获取插值后的目标位置和速度
                float target_position = interpolated_positions[i];
                float target_velocity = (i < interpolated_velocities.size()) ? interpolated_velocities[i] : 0.0f;

                // 获取当前位置（从状态反馈）- 关键改进！
                float current_position = motor_[motor_index].q;
                float current_velocity = motor_[motor_index].dq;

                // PD控制计算补偿力矩（减少抖动）
                float position_error = target_position - current_position;
                float velocity_error = target_velocity - current_velocity;
                float tau_compensation = 5.0f * position_error + 0.1f * velocity_error;

                RCLCPP_DEBUG(this->get_logger(), "%s 关节 %s: 目标=%.3f, 当前=%.3f, 误差=%.3f",
                            arm_label.c_str(), joint_name.c_str(), target_position, current_position, position_error);

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

  // 左臂 Action 服务器和轨迹执行状态
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_left_;
  std::shared_ptr<GoalHandle> goal_handle_left_;
  trajectory_msgs::msg::JointTrajectory current_trajectory_left_;
  size_t current_trajectory_point_index_left_;
  double trajectory_start_time_left_;
  bool executing_trajectory_left_;

  // 右臂 Action 服务器和轨迹执行状态
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_right_;
  std::shared_ptr<GoalHandle> goal_handle_right_;
  trajectory_msgs::msg::JointTrajectory current_trajectory_right_;
  size_t current_trajectory_point_index_right_;
  double trajectory_start_time_right_;
  bool executing_trajectory_right_;


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
