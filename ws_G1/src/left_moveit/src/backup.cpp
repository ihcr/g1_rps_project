/**
 * Real G1 DEX3-1 Gripper Control for Rock-Paper-Scissors
 *
 * 这个程序订阅 /rps/ai_move 话题，根据收到的命令(rock/paper/scissors)
 * 通过 unitree_hg::msg::HandCmd 控制真实的G1左手夹爪做出对应手势
 *
 * 基于官方 g1_dex3_example.cpp 改编
 **/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "unitree_hg/msg/hand_cmd.hpp"
#include "unitree_hg/msg/hand_state.hpp"

#include <algorithm>
#include <cctype>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

// DEX3-1 夹爪参数
constexpr int32_t MOTOR_MAX = 7;
constexpr int32_t SENSOR_MAX = 9;

// 左手关节限位 (根据官方示例)
const float maxLimits_left[7] = {1.05, 1.05, 1.75, 0, 0, 0, 0};
const float minLimits_left[7] = {-1.05, -0.724, 0, -1.57, -1.75, -1.57, -1.75};

// ROS2 话题名称
const std::string HANDCMD_TOPIC = "dex3/left/cmd";
const std::string HANDSTATE_TOPIC = "lf/dex3/left/state";
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

    // 兼容中文
    if (s == "石头" || s == "拳头" || s == "拳") return "rock";
    if (s == "布") return "paper";
    if (s == "剪刀") return "scissors";

    return "";
}

// ========================
// 主控制节点
// ========================
class RealGripperController : public rclcpp::Node
{
public:
    RealGripperController() : Node("real_gripper_controller")
    {
        // 发布命令到真实夹爪
        handcmd_publisher_ = this->create_publisher<unitree_hg::msg::HandCmd>(HANDCMD_TOPIC, 10);

        // 订阅夹爪状态（可选，用于监控）
        handstate_subscriber_ = this->create_subscription<unitree_hg::msg::HandState>(
            HANDSTATE_TOPIC, 10,
            std::bind(&RealGripperController::handStateCallback, this, std::placeholders::_1));

        // 订阅石头剪刀布命令
        rps_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            RPS_TOPIC, 10,
            std::bind(&RealGripperController::rpsCallback, this, std::placeholders::_1));

        // 定义三种手势的关节目标位置
        initializeGestures();

        // 防抖参数
        last_executed_cmd_ = "";
        last_exec_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "====== Real DEX3-1 Gripper Controller Started ======");
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", HANDCMD_TOPIC.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", RPS_TOPIC.c_str());
        RCLCPP_INFO(this->get_logger(), "Ready to receive rock/paper/scissors commands!");
    }

private:
    void initializeGestures()
    {
        // DEX3-1 左手关节顺序：
        // 0: thumb_0 (大拇指根部)
        // 1: thumb_1 (大拇指中部)
        // 2: thumb_2 (大拇指尖部)
        // 3: index_0 (食指)
        // 4: index_1 (食指)
        // 5: middle_0 (中指/无名指/小指组)
        // 6: middle_1 (中指/无名指/小指组)

        // 布 - 所有手指收拢
        gesture_positions_["paper"] = {
            -0.0,   // thumb_0: 收拢
            -0.5,   // thumb_1: 收拢
            0.0,    // thumb_2: 弯曲
            0.0,   // index_0: 收拢
            0.0,    // index_1: 保持
            0.0,    // middle_0: 收拢
            0.0     // middle_1: 收拢
        };

        // 石头 - 所有手指伸展
        gesture_positions_["rock"] = {
            0.8,    // thumb_0: 伸展
            0.5,    // thumb_1: 伸展
            0.8,    // thumb_2: 伸直
            -1.2,    // index_0: 伸展
            -1.2,    // index_1: 伸直
            0.8,    // middle_0: 伸展
            0.8     // middle_1: 伸直
        };

        // 剪刀 - 食指中指伸展，其他收拢
        gesture_positions_["scissors"] = {
            0.8,   // thumb_0: 收拢
            0.5,   // thumb_1: 收拢
            0.8,    // thumb_2: 弯曲
            0.0,    // index_0: 伸展
            0.0,    // index_1: 伸直
            -0.5,    // middle_0: 收拢（无名指小指）
            -0.5     // middle_1: 收拢
        };

        RCLCPP_INFO(this->get_logger(), "Gesture positions initialized (rock/paper/scissors)");
    }

    void handStateCallback(unitree_hg::msg::HandState::SharedPtr msg)
    {
        // 保存夹爪状态用于监控（可选）
        std::lock_guard<std::mutex> lock(state_mutex_);
        hand_state_ = msg;
    }

    void rpsCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd_raw = msg->data;
        std::string cmd = normalize_cmd(cmd_raw);

        if (cmd.empty()) {
            RCLCPP_WARN(this->get_logger(), "Unknown command '%s' (expect rock/paper/scissors). Ignored.",
                        cmd_raw.c_str());
            return;
        }

        // 防抖：相同命令0.5s内重复则忽略
        auto now = this->now();
        if (cmd == last_executed_cmd_ && (now - last_exec_time_).seconds() < 0.5) {
            RCLCPP_WARN(this->get_logger(), "Debounced duplicate cmd='%s'", cmd.c_str());
            return;
        }

        // 查找手势位置
        auto it = gesture_positions_.find(cmd);
        if (it == gesture_positions_.end()) {
            RCLCPP_ERROR(this->get_logger(), "No gesture defined for cmd='%s'", cmd.c_str());
            return;
        }

        // 执行手势
        RCLCPP_INFO(this->get_logger(), "Executing gesture: %s", cmd.c_str());
        executeGesture(it->second);

        // 更新防抖记录
        last_executed_cmd_ = cmd;
        last_exec_time_ = now;
    }

    void executeGesture(const std::vector<float>& target_positions)
    {
        if (target_positions.size() != MOTOR_MAX) {
            RCLCPP_ERROR(this->get_logger(), "Invalid gesture positions size: %zu (expected %d)",
                         target_positions.size(), MOTOR_MAX);
            return;
        }

        // 构建HandCmd消息
        unitree_hg::msg::HandCmd msg;
        msg.motor_cmd.resize(MOTOR_MAX);

        for (int i = 0; i < MOTOR_MAX; i++)
        {
            // 设置控制模式
            RIS_Mode_t ris_mode;
            ris_mode.id = i;
            ris_mode.status = 0x01;  // Position control mode
            ris_mode.timeout = 0;

            uint8_t mode = 0;
            mode |= (ris_mode.id & 0x0F);
            mode |= (ris_mode.status & 0x07) << 4;
            mode |= (ris_mode.timeout & 0x01) << 7;

            msg.motor_cmd[i].set__mode(mode);
            msg.motor_cmd[i].set__q(target_positions[i]);  // 目标位置
            msg.motor_cmd[i].set__dq(0.0);                  // 目标速度
            msg.motor_cmd[i].set__tau(0.0);                 // 前馈力矩
            msg.motor_cmd[i].set__kp(1.5);                  // 位置增益
            msg.motor_cmd[i].set__kd(0.15);                 // 阻尼增益
        }

        // 发布命令
        handcmd_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Gesture command published to real gripper");
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

    auto node = std::make_shared<RealGripperController>();

    RCLCPP_INFO(node->get_logger(), "Spinning... Press Ctrl+C to exit.");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
