/**
 * Real G1 DEX3-1 Gripper Control for Rock-Paper-Scissors (RIGHT HAND)
 *
 * 这个程序订阅 /rps/ai_move 话题，根据收到的命令(rock/paper/scissors)
 * 通过 unitree_hg::msg::HandCmd 控制真实的G1右手夹爪做出对应手势
 *
 * 基于 real_gripper_control.cpp (左手版本) 改编为右手
 **/

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

// DEX3-1 夹爪参数
constexpr int32_t MOTOR_MAX = 7;
constexpr int32_t SENSOR_MAX = 9;

// 右手关节限位 (根据官方示例)
const float maxLimits_right[7] = {1.05, 0.742, 0, 1.57, 1.75, 1.57, 1.75};
const float minLimits_right[7] = {-1.05, -1.05, -1.75, 0, 0, 0, 0};

// ROS2 话题名称 - 右手
const std::string HANDCMD_TOPIC = "dex3/right/cmd";
const std::string HANDSTATE_TOPIC = "lf/dex3/right/state";
const std::string RPS_TOPIC = "/rps/ai_move";

// ---- 工具函数 ----
typedef struct
{
    uint8_t id : 4;
    uint8_t status : 3;
    uint8_t timeout : 1;
} RIS_Mode_t;

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

// 归一化命令
static inline std::string normalize_cmd(const std::string& raw)
{
    std::string s = trim_copy(raw);
    std::string sl = tolower_copy(s);

    if (sl == "rock" || sl == "stone" || sl == "fist") return "rock";
    if (sl == "paper") return "paper";
    if (sl == "scissors" || sl == "scissor") return "scissors";
    if (sl == "ready" || sl == "idle" || sl == "wait") return "ready";

    // 兼容中文
    if (s == "石头" || s == "拳头" || s == "拳") return "rock";
    if (s == "布") return "paper";
    if (s == "剪刀") return "scissors";
    if (s == "准备" || s == "待机") return "ready";

    return "";
}

template <typename T>
static inline T clamp_val(T v, T lo, T hi)
{
    return std::max(lo, std::min(v, hi));
}

// ========================
// 主控制节点
// ========================
class RealGripperControllerRight : public rclcpp::Node
{
public:
    RealGripperControllerRight() : Node("real_gripper_controller_right")
    {
        // 发布命令到真实夹爪
        handcmd_publisher_ = this->create_publisher<unitree_hg::msg::HandCmd>(HANDCMD_TOPIC, 10);

        // 订阅夹爪状态（可选，用于监控）
        handstate_subscriber_ = this->create_subscription<unitree_hg::msg::HandState>(
            HANDSTATE_TOPIC, 10,
            std::bind(&RealGripperControllerRight::handStateCallback, this, std::placeholders::_1));

        // 订阅石头剪刀布命令
        rps_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            RPS_TOPIC, 10,
            std::bind(&RealGripperControllerRight::rpsCallback, this, std::placeholders::_1));

        // 定义三种手势的关节目标位置
        initializeGestures();

        // 防抖参数
        last_executed_cmd_ = "";
        last_exec_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "====== Real DEX3-1 RIGHT Hand Gripper Controller Started ======");
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", HANDCMD_TOPIC.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", RPS_TOPIC.c_str());
        RCLCPP_INFO(this->get_logger(), "Ready to receive rock/paper/scissors commands!");
    }

private:
    void initializeGestures()
    {
        // DEX3-1 右手关节顺序：
        // 0: thumb_0
        // 1: thumb_1
        // 2: thumb_2
        // 3: index_0
        // 4: index_1
        // 5: middle_0
        // 6: middle_1

        // 与左手版本 real_gripper_control.cpp 保持一致：
        // 1) 不硬编码超范围目标值；
        // 2) 所有目标都由关节限位(min/max)推导；
        // 3) thumb_0、thumb_1 采用中位值，避免贴限位造成过热/抖动。

        auto mid  = [](int i) -> float { return 0.5f * (maxLimits_right[i] + minLimits_right[i]); };
        auto qmin = [](int i) -> float { return minLimits_right[i]; };
        auto qmax = [](int i) -> float { return maxLimits_right[i]; };

        // 右手的“张开/握拳”方向与左手不同：
        // - joints 3-6: [0, +]，0 更接近张开，正值更接近弯曲
        // - joint 2:    [-, 0]，更负更接近张开，0 更接近弯曲

        // paper: 尽量张开
        gesture_positions_["paper"] = {
            mid(0),    // thumb_0
            qmax(1),    // thumb_1
            qmax(2),   // thumb_2: 更负更张开
            qmin(3),   // index_0: 0 更张开
            qmin(4),
            qmin(5),
            qmin(6)
        };

        // rock: 尽量握拳
        gesture_positions_["rock"] = {
      mid(0),
      mid(1),
      qmin(2),
      qmax(3),
      qmax(4),
      qmax(5),
      qmax(6)
        };

        // scissors: 食指张开，其它收拢
        gesture_positions_["scissors"] = {
      mid(0),
      qmin(1),
      qmin(2),
      qmin(3),
      qmin(4),
      qmin(5),
      qmin(6)
        };

        // ready: 初始待机姿势（所有手指半张开，自然放松状态）
        // 与石头剪刀布都不同，用于等待下一轮游戏
        gesture_positions_["ready"] = {
            mid(0),    // thumb_0: 中位
            mid(1),    // thumb_1: 中位
            mid(2),    // thumb_2: 中位
            mid(3),    // index_0: 中位（半弯曲）
            mid(4),    // index_1: 中位
            mid(5),    // middle_0: 中位
            mid(6)     // middle_1: 中位
        };

        RCLCPP_INFO(this->get_logger(), "RIGHT hand gesture positions initialized (rock/paper/scissors/ready)");
        RCLCPP_INFO(this->get_logger(), "Note: Right hand joints 3-6 use POSITIVE values (opposite of left hand)");
    }

    void handStateCallback(const unitree_hg::msg::HandState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        hand_state_ = msg;
    }

    void rpsCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd_raw = msg->data;
        std::string cmd = normalize_cmd(cmd_raw);

        if (cmd.empty()) {
            RCLCPP_WARN(this->get_logger(), "Unknown command '%s' (expect rock/paper/scissors/ready). Ignored.",
                        cmd_raw.c_str());
            return;
        }

        auto now = this->now();
        if (cmd == last_executed_cmd_ && (now - last_exec_time_).seconds() < 0.5) {
            RCLCPP_WARN(this->get_logger(), "Debounced duplicate cmd='%s'", cmd.c_str());
            return;
        }

        auto it = gesture_positions_.find(cmd);
        if (it == gesture_positions_.end()) {
            RCLCPP_ERROR(this->get_logger(), "No gesture defined for cmd='%s'", cmd.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Executing gesture: %s", cmd.c_str());
        executeGesture(it->second);

        last_executed_cmd_ = cmd;
        last_exec_time_ = now;
    }

    void executeGesture(const std::vector<float>& target_positions)
    {
        if (target_positions.size() != static_cast<size_t>(MOTOR_MAX)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid gesture positions size: %zu (expected %d)",
                         target_positions.size(), MOTOR_MAX);
            return;
        }

        // 1) Clamp target within joint limits
        std::vector<float> target_q(MOTOR_MAX, 0.0f);
        for (int i = 0; i < MOTOR_MAX; ++i) {
            target_q[i] = clamp_val(target_positions[i], minLimits_right[i], maxLimits_right[i]);
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

        // Helper to publish one command frame (same style as left-hand controller)
        auto publish_cmd = [&](const std::vector<float>& q_des, float kp, float kd) {
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

                const float q = clamp_val(q_des[i], minLimits_right[i], maxLimits_right[i]);

                cmd_msg.motor_cmd[i].set__mode(mode);
                cmd_msg.motor_cmd[i].set__q(q);
                cmd_msg.motor_cmd[i].set__dq(0.0f);
                cmd_msg.motor_cmd[i].set__tau(0.0f);
                cmd_msg.motor_cmd[i].set__kp(kp);
                cmd_msg.motor_cmd[i].set__kd(kd);
            }

            handcmd_publisher_->publish(cmd_msg);
        };

        // 3) Smooth move phase (soft gains)
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

        // 4) Hold phase (stiffer gains)
        publish_cmd(target_q, 1.5f, 0.1f);

        RCLCPP_INFO(this->get_logger(), "Gesture command published to RIGHT hand gripper");
    }

    // 成员变量
    rclcpp::Publisher<unitree_hg::msg::HandCmd>::SharedPtr handcmd_publisher_;
    rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr handstate_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rps_subscriber_;

    std::unordered_map<std::string, std::vector<float>> gesture_positions_;

    std::string last_executed_cmd_;
    rclcpp::Time last_exec_time_;

    std::mutex state_mutex_;
    unitree_hg::msg::HandState::SharedPtr hand_state_;
};

// ========================
// 主函数
// ========================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RealGripperControllerRight>();

    RCLCPP_INFO(node->get_logger(), "Spinning... Press Ctrl+C to exit.");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

