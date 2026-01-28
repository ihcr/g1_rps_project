#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <algorithm>
#include <cctype>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// ---- 小工具：trim + tolower ----
static inline void ltrim(std::string& s) {
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
}
static inline void rtrim(std::string& s) {
  s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
}
static inline std::string trim_copy(std::string s) {
  ltrim(s); rtrim(s); return s;
}
static inline std::string tolower_copy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (unsigned char)std::tolower(c); });
  return s;
}

// ---- 将输入命令归一化为 rock/paper/scissors ----
static inline std::string normalize_cmd(const std::string& raw)
{
  std::string s = trim_copy(raw);
  // 常见情况：英文
  std::string sl = tolower_copy(s);

  if (sl == "rock" || sl == "stone" || sl == "fist") return "rock";
  if (sl == "paper") return "paper";
  if (sl == "scissors" || sl == "scissor") return "scissors";

  // 兼容中文
  if (s == "石头" || s == "拳头" || s == "拳") return "rock";
  if (s == "布") return "paper";
  if (s == "剪刀") return "scissors";

  return ""; // unknown
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("gripper_control_rps_sub");

  // 单独起 executor spin（保持你原来的结构）
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&exec]() { exec.spin(); });

  // MoveGroupInterface for gripper (DEX3-1)
  static const std::string PLANNING_GROUP_GRIPPER = "left_gripper";
  moveit::planning_interface::MoveGroupInterface move_group_gripper(node, PLANNING_GROUP_GRIPPER);

  move_group_gripper.setMaxVelocityScalingFactor(1.0);
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);

  // （可选）减少规划拖延：让 plan 更快返回
  move_group_gripper.setPlanningTime(0.2);
  move_group_gripper.setNumPlanningAttempts(1);



  std::cout << "\n====== DEX3-1 夹爪控制 - ROS订阅石头剪刀布 ======\n" << std::endl;
  RCLCPP_INFO(node->get_logger(), "Gripper control ready! Subscribing: /rps/ai_move (std_msgs/String)");

  // ------------------------------------------------------------
  // 你原来的三种手势关节位置（保持不动）
  // 关节顺序: thumb_0, thumb_1, thumb_2, index_0, index_1, middle_0, middle_1
  // ------------------------------------------------------------

  // 1) 石头
  std::vector<double> rock_positions = {
    -1.5,   // left_hand_thumb_0_joint
    -1.5,   // left_hand_thumb_1_joint
    -1.5,   // left_hand_thumb_2_joint
    -1.5,   // left_hand_index_0_joint
    0.0,    // left_hand_index_1_joint
    1.0,    // left_hand_middle_0_joint
    1.0     // left_hand_middle_1_joint
  };

  // 2) 剪刀
  std::vector<double> scissors_positions = {
    0.0,   // left_hand_thumb_0_joint
    0.0,   // left_hand_thumb_1_joint
    0.0,   // left_hand_thumb_2_joint
    0.0,   // left_hand_index_0_joint
    0.0,   // left_hand_index_1_joint
    1.0,   // left_hand_middle_0_joint
    1.0    // left_hand_middle_1_joint
  };

  // 3) 布
  std::vector<double> paper_positions = {
    0.0,   // left_hand_thumb_0_joint
    0.0,   // left_hand_thumb_1_joint
    0.0,   // left_hand_thumb_2_joint
    0.0,   // left_hand_index_0_joint
    0.0,   // left_hand_index_1_joint
    0.0,   // left_hand_middle_0_joint
    0.0    // left_hand_middle_1_joint
  };

  // 命令 -> 关节目标
  std::unordered_map<std::string, std::vector<double>> pose_map = {
    {"rock", rock_positions},
    {"paper", paper_positions},
    {"scissors", scissors_positions},
  };

  // ------------------------------------------------------------
  // ROS订阅：只缓存“最新命令”，主线程负责执行 plan/execute
  // ------------------------------------------------------------
  std::mutex cmd_mtx;
  std::string pending_cmd_raw;
  rclcpp::Time pending_stamp = node->now();

  auto sub = node->create_subscription<std_msgs::msg::String>(
    "/rps/ai_move", 10,
    [&](const std_msgs::msg::String::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(cmd_mtx);
      pending_cmd_raw = msg->data;
      pending_stamp = node->now();
      RCLCPP_INFO(node->get_logger(), "Received /rps/ai_move: '%s'", pending_cmd_raw.c_str());
    }
  );

  // 去抖：避免同一个指令短时间反复触发
  std::string last_executed;
  rclcpp::Time last_exec_time = node->now();

  rclcpp::WallRate rate(50); // 50 Hz 检查是否有新命令
  while (rclcpp::ok()) {

    std::string cmd_raw;
    rclcpp::Time cmd_time;

    {
      std::lock_guard<std::mutex> lk(cmd_mtx);
      cmd_raw = pending_cmd_raw;
      cmd_time = pending_stamp;
      // 取出来后清空，保证“触发一次执行一次”
      pending_cmd_raw.clear();
    }

    if (!cmd_raw.empty()) {
      std::string cmd = normalize_cmd(cmd_raw);
      if (cmd.empty()) {
        RCLCPP_WARN(node->get_logger(),
                    "Unknown cmd '%s' (expect rock/paper/scissors or 石头/剪刀/布). Ignored.",
                    cmd_raw.c_str());
      } else {
        // debounce：同指令0.5s内重复则忽略
        auto now = node->now();
        if (cmd == last_executed && (now - last_exec_time).seconds() < 0.5) {
          RCLCPP_WARN(node->get_logger(), "Debounced duplicate cmd='%s'", cmd.c_str());
        } else {
          auto it = pose_map.find(cmd);
          if (it == pose_map.end()) {
            RCLCPP_ERROR(node->get_logger(), "Internal map missing cmd='%s'", cmd.c_str());
          } else {
            RCLCPP_INFO(node->get_logger(), "Executing gesture for cmd='%s' ...", cmd.c_str());

            move_group_gripper.setJointValueTarget(it->second);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_gripper.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (success) {
              move_group_gripper.execute(plan);
              RCLCPP_INFO(node->get_logger(), "Gesture cmd='%s' executed OK.", cmd.c_str());
              last_executed = cmd;
              last_exec_time = now;
            } else {
              RCLCPP_ERROR(node->get_logger(), "Planning failed for cmd='%s'.", cmd.c_str());
            }
          }
        }
      }
    }

    rate.sleep();
  }

  // 收尾
  exec.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  rclcpp::shutdown();
  return 0;
}

