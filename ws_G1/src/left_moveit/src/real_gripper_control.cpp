/**
 * Real G1 DEX3-1 Gripper Control for Rock-Paper-Scissors
 *
 * Subscribes:  /rps/ai_move  (std_msgs/String: rock|paper|scissors)
 * Publishes:   dex3/left/cmd (unitree_hg/HandCmd)
 * Subscribes:  lf/dex3/left/state (unitree_hg/HandState)  [optional, for smoothing]
 *
 * NOTE: This version fixes brace / scope issues and keeps the original node logic,
 * while making the gripper motion command style consistent with g1_dex3_example*.cpp:
 * - mode: position, timeout=0, tau=0
 * - dq = 0 (desired final vel)
 * - soft move gain then hold gain
 * - always clamp q within joint limits
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "unitree_hg/msg/hand_cmd.hpp"
#include "unitree_hg/msg/hand_state.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// DEX3-1 parameters
constexpr int32_t MOTOR_MAX = 7;

// ROS2 topic names
const std::string HANDCMD_TOPIC   = "dex3/left/cmd";
const std::string HANDSTATE_TOPIC = "lf/dex3/left/state";
const std::string RPS_TOPIC       = "/rps/ai_move";

// Left-hand joint limits (from example)
const float maxLimits_left[MOTOR_MAX] = {1.05f, 1.05f, 1.75f, 0.0f, 0.0f, 0.0f, 0.0f};
const float minLimits_left[MOTOR_MAX] = {-1.05f, -0.724f, 0.0f, -1.57f, -1.75f, -1.57f, -1.75f};

// Packed mode bitfield
typedef struct
{
  uint8_t id : 4;
  uint8_t status : 3;
  uint8_t timeout : 1;
} RIS_Mode_t;

// --- Small utilities (C++14-friendly) ---
static inline void ltrim(std::string &s)
{
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
}

static inline void rtrim(std::string &s)
{
  s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
}

static inline std::string trim_copy(std::string s)
{
  ltrim(s);
  rtrim(s);
  return s;
}

static inline std::string tolower_copy(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

static inline std::string normalize_cmd(const std::string &raw)
{
  std::string s = tolower_copy(trim_copy(raw));
  if (s == "rock" || s == "paper" || s == "scissors") return s;
  if (s == "r") return "rock";
  if (s == "p") return "paper";
  if (s == "s") return "scissors";
  return "";
}

template <typename T>
static inline T clamp_val(T v, T lo, T hi)
{
  return std::max(lo, std::min(v, hi));
}

// ========================
// Gripper control node
// ========================
class RealGripperController : public rclcpp::Node
{
public:
  RealGripperController() : Node("real_gripper_controller")
  {
    // Publisher: command
    handcmd_publisher_ = this->create_publisher<unitree_hg::msg::HandCmd>(HANDCMD_TOPIC, 10);

    // Subscriber: state (optional)
    handstate_subscriber_ = this->create_subscription<unitree_hg::msg::HandState>(
      HANDSTATE_TOPIC, 10,
      std::bind(&RealGripperController::handStateCallback, this, std::placeholders::_1));

    // Subscriber: RPS command
    rps_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      RPS_TOPIC, 10,
      std::bind(&RealGripperController::rpsCallback, this, std::placeholders::_1));

    initializeGestures();

    last_executed_cmd_.clear();
    last_exec_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "====== Real DEX3-1 Gripper Controller Started ======");
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", HANDCMD_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", RPS_TOPIC.c_str());
    RCLCPP_INFO(this->get_logger(), "Ready: rock / paper / scissors");
  }

private:
  void initializeGestures()
  {
    // Keep gesture definition logic; values derived from limits (no hard-coded out-of-range targets).
    auto mid = [](int i) -> float { return 0.5f * (maxLimits_left[i] + minLimits_left[i]); };
    auto qmin = [](int i) -> float { return minLimits_left[i]; };
    auto qmax = [](int i) -> float { return maxLimits_left[i]; };

    // paper: as open as possible (towards max)
    gesture_positions_["paper"] = {
      mid(0),
      mid(1),
      qmin(2),
      qmax(3),
      qmax(4),
      qmax(5),
      qmax(6)
    };

    // rock: as closed as possible (towards min)
    gesture_positions_["rock"] = {
      mid(0),
      mid(1),
      qmax(2),
      qmin(3),
      qmin(4),
      qmin(5),
      qmin(6)
    };

    // scissors: index open, others closed
    gesture_positions_["scissors"] = {
      mid(0),
      mid(1),
      qmax(2),
      qmax(3),
      qmax(4),
      qmax(5),
      qmax(6)
    };
  }

  void handStateCallback(const unitree_hg::msg::HandState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    hand_state_ = msg;
  }

  void rpsCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string cmd_raw = msg->data;
    const std::string cmd = normalize_cmd(cmd_raw);

    if (cmd.empty()) {
      RCLCPP_WARN(this->get_logger(), "Unknown command '%s' (expect rock/paper/scissors). Ignored.", cmd_raw.c_str());
      return;
    }

    // Debounce: ignore duplicates within 0.5s
    const auto now = this->now();
    if (cmd == last_executed_cmd_ && (now - last_exec_time_).seconds() < 0.5) {
      RCLCPP_WARN(this->get_logger(), "Debounced duplicate cmd='%s'", cmd.c_str());
      return;
    }

    const auto it = gesture_positions_.find(cmd);
    if (it == gesture_positions_.end()) {
      RCLCPP_ERROR(this->get_logger(), "No gesture defined for cmd='%s'", cmd.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Executing gesture: %s", cmd.c_str());
    executeGesture(it->second);

    last_executed_cmd_ = cmd;
    last_exec_time_ = now;
  }

  void executeGesture(const std::vector<float> &target_positions)
  {
    if (target_positions.size() != static_cast<size_t>(MOTOR_MAX)) {
      RCLCPP_ERROR(this->get_logger(), "Invalid gesture positions size: %zu (expected %d)",
                   target_positions.size(), MOTOR_MAX);
      return;
    }

    // 1) Clamp target within limits
    std::vector<float> target_q(MOTOR_MAX, 0.0f);
    for (int i = 0; i < MOTOR_MAX; ++i) {
      target_q[i] = clamp_val(target_positions[i], minLimits_left[i], maxLimits_left[i]);
    }

    // 2) Read current q (if available) for smooth interpolation
    std::vector<float> start_q = target_q;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (hand_state_ && hand_state_->motor_state.size() >= static_cast<size_t>(MOTOR_MAX)) {
        for (int i = 0; i < MOTOR_MAX; ++i) {
          start_q[i] = hand_state_->motor_state[i].q;
        }
      }
    }

    // Helper to publish one command frame (consistent with example)
    auto publish_cmd = [&](const std::vector<float> &q_des, float kp, float kd) {
      unitree_hg::msg::HandCmd cmd_msg;
      cmd_msg.motor_cmd.resize(MOTOR_MAX);

      for (int i = 0; i < MOTOR_MAX; ++i) {
        RIS_Mode_t ris_mode;
        ris_mode.id = static_cast<uint8_t>(i);
        ris_mode.status = 0x01;  // position control
        ris_mode.timeout = 0;

        uint8_t mode = 0;
        mode |= (ris_mode.id & 0x0F);
        mode |= (ris_mode.status & 0x07) << 4;
        mode |= (ris_mode.timeout & 0x01) << 7;

        const float q = clamp_val(q_des[i], minLimits_left[i], maxLimits_left[i]);

        cmd_msg.motor_cmd[i].set__mode(mode);
        cmd_msg.motor_cmd[i].set__q(q);
        cmd_msg.motor_cmd[i].set__dq(0.0f);
        cmd_msg.motor_cmd[i].set__tau(0.0f);
        cmd_msg.motor_cmd[i].set__kp(kp);
        cmd_msg.motor_cmd[i].set__kd(kd);
      }

      handcmd_publisher_->publish(cmd_msg);
    };

    // 3) Smooth move phase (soft gains like rotateMotors)
    const int steps = 30;
    const auto dt = std::chrono::milliseconds(10);
    std::vector<float> q_step(MOTOR_MAX, 0.0f);

    for (int s = 1; s <= steps && rclcpp::ok(); ++s) {
      const float t = static_cast<float>(s) / static_cast<float>(steps);
      for (int i = 0; i < MOTOR_MAX; ++i) {
        q_step[i] = start_q[i] + t * (target_q[i] - start_q[i]);
      }
      publish_cmd(q_step, 0.5f, 0.1f);
      std::this_thread::sleep_for(dt);
    }

    // 4) Hold phase (stiffer gains like gripHand)
    publish_cmd(target_q, 1.5f, 0.1f);

    RCLCPP_INFO(this->get_logger(), "Gesture command published to real gripper");
  }

private:
  rclcpp::Publisher<unitree_hg::msg::HandCmd>::SharedPtr handcmd_publisher_;
  rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr handstate_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rps_subscriber_;

  std::unordered_map<std::string, std::vector<float>> gesture_positions_;

  std::string last_executed_cmd_;
  rclcpp::Time last_exec_time_;

  std::mutex state_mutex_;
  unitree_hg::msg::HandState::SharedPtr hand_state_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealGripperController>();
  RCLCPP_INFO(node->get_logger(), "Spinning... Press Ctrl+C to exit.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
