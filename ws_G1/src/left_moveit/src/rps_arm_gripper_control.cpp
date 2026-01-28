/**
 * 石头剪刀布 - 手臂运动 + 夹爪控制整合版
 *
 * 功能：
 * 1. 订阅 /rps/ai_move 话题（rock/paper/scissors）
 * 2. 控制左臂移动到游戏位置（基于g1_arm7_control.cpp）
 * 3. 控制左手夹爪做出对应手势（基于real_gripper_control.cpp）
 *
 * 结合了：
 * - g1_arm7_control.cpp (手臂关节控制)
 * - real_gripper_control.cpp (夹爪手势控制)
 **/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/hand_cmd.hpp"
#include "unitree_hg/msg/hand_state.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// ============ 常量定义 ============
constexpr float kPi = 3.141592654;
constexpr float kPi_2 = 1.57079632;
constexpr int32_t MOTOR_MAX = 7;  // 夹爪电机数

// 手臂关节索引（从g1_arm7_control.cpp）
enum JointIndex
{
    kLeftHipPitch, kLeftHipRoll, kLeftHipYaw, kLeftKnee, kLeftAnkle, kLeftAnkleRoll,
    kRightHipPitch, kRightHipRoll, kRightHipYaw, kRightKnee, kRightAnkle, kRightAnkleRoll,
    kWaistYaw, kWaistRoll, kWaistPitch,
    kLeftShoulderPitch, kLeftShoulderRoll, kLeftShoulderYaw, kLeftElbow,
    kLeftWistRoll, kLeftWistPitch, kLeftWistYaw,
    kRightShoulderPitch, kRightShoulderRoll, kRightShoulderYaw, kRightElbow,
    kRightWistRoll, kRightWistPitch, kRightWistYaw,
    kNotUsedJoint, kNotUsedJoint1, kNotUsedJoint2, kNotUsedJoint3, kNotUsedJoint4, kNotUsedJoint5
};

// ROS话题名称
const std::string ARM_CMD_TOPIC = "/arm_sdk";
const std::string ARM_STATE_TOPIC = "lowstate";
const std::string HAND_CMD_TOPIC = "dex3/left/cmd";
const std::string HAND_STATE_TOPIC = "lf/dex3/left/state";
const std::string RPS_CMD_TOPIC = "/rps/ai_move";

// 夹爪关节限位
const float maxLimits_left[7] = {1.05, 1.05, 1.75, 0, 0, 0, 0};
const float minLimits_left[7] = {-1.05, -0.724, 0, -1.57, -1.75, -1.57, -1.75};

// ============ 工具函数 ============
typedef struct {
    uint8_t id : 4;
    uint8_t status : 3;
    uint8_t timeout : 1;
} RIS_Mode_t;

static inline std::string trim_copy(std::string s) {
    auto ltrim = [](std::string& str) {
        str.erase(str.begin(), std::find_if(str.begin(), str.end(), [](unsigned char ch) {
            return !std::isspace(ch);
        }));
    };
    auto rtrim = [](std::string& str) {
        str.erase(std::find_if(str.rbegin(), str.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
        }).base(), str.end());
    };
    ltrim(s); rtrim(s);
    return s;
}

static inline std::string normalize_cmd(const std::string& raw) {
    std::string s = trim_copy(raw);
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);

    if (s == "rock" || s == "stone" || s == "fist" || s == "石头") return "rock";
    if (s == "paper" || s == "布") return "paper";
    if (s == "scissors" || s == "scissor" || s == "剪刀") return "scissors";
    return "";
}

// ============ 主控制器 ============
class RPSArmGripperController : public rclcpp::Node
{
public:
    RPSArmGripperController() : Node("rps_arm_gripper_controller")
    {
        // 发布器
        arm_cmd_pub_ = this->create_publisher<unitree_hg::msg::LowCmd>(ARM_CMD_TOPIC, 10);
        hand_cmd_pub_ = this->create_publisher<unitree_hg::msg::HandCmd>(HAND_CMD_TOPIC, 10);

        // 订阅器
        arm_state_sub_ = this->create_subscription<unitree_hg::msg::LowState>(
            ARM_STATE_TOPIC, 10,
            std::bind(&RPSArmGripperController::armStateCallback, this, std::placeholders::_1));

        hand_state_sub_ = this->create_subscription<unitree_hg::msg::HandState>(
            HAND_STATE_TOPIC, 10,
            std::bind(&RPSArmGripperController::handStateCallback, this, std::placeholders::_1));

        rps_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            RPS_CMD_TOPIC, 10,
            std::bind(&RPSArmGripperController::rpsCommandCallback, this, std::placeholders::_1));

        // 初始化手臂和夹爪配置
        initializeArmPositions();
        initializeGripperGestures();

        // 控制参数
        control_dt_ = 0.02f;  // 20ms控制周期
        kp_arm_ = 60.f;
        kd_arm_ = 1.5f;
        kp_gripper_ = 1.5f;
        kd_gripper_ = 0.15f;

        is_arm_ready_ = false;
        last_cmd_ = "";
        last_cmd_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "===== RPS Arm+Gripper Controller Started =====");
        RCLCPP_INFO(this->get_logger(), "Waiting for robot state...");
    }

private:
    // ========== 初始化配置 ==========
    void initializeArmPositions()
    {
        // 左臂关节索引（只控制左臂7个关节）
        left_arm_joints_ = {
            JointIndex::kLeftShoulderPitch,
            JointIndex::kLeftShoulderRoll,
            JointIndex::kLeftShoulderYaw,
            JointIndex::kLeftElbow,
            JointIndex::kLeftWistRoll,
            JointIndex::kLeftWistPitch,
            JointIndex::kLeftWistYaw
        };

        // 手臂待机位置（放下）
        arm_home_pos_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        // 手臂游戏位置（举起，便于摄像头看到）
        // LeftShoulderPitch, LeftShoulderRoll, LeftShoulderYaw, LeftElbow,
        // LeftWistRoll, LeftWistPitch, LeftWistYaw
        arm_game_pos_ = {
            -0.3f,   // ShoulderPitch: 向前抬起
            kPi_2,   // ShoulderRoll: 外展90度
            0.0f,    // ShoulderYaw: 中性
            kPi_2,   // Elbow: 弯曲90度
            0.0f,    // WistRoll: 中性
            0.0f,    // WistPitch: 中性
            0.0f     // WistYaw: 中性
        };

        current_arm_pos_ = arm_home_pos_;
    }

    void initializeGripperGestures()
    {
        // 石头 - 所有手指收拢（参考g1_dex3_example.cpp限位）
        gripper_gestures_["rock"] = {
            0.0f,    // thumb_0
            0.0f,    // thumb_1
            1.3f,    // thumb_2: 弯曲
            -1.2f,   // index_0: 收拢（负值）
            -1.4f,   // index_1: 收拢
            -1.2f,   // middle_0: 收拢
            -1.4f    // middle_1: 收拢
        };

        // 布 - 所有手指伸展
        gripper_gestures_["paper"] = {
            0.5f,    // thumb_0: 伸展
            0.5f,    // thumb_1: 伸展
            0.2f,    // thumb_2: 略微弯曲
            0.0f,    // index_0: 伸直
            0.0f,    // index_1: 伸直
            0.0f,    // middle_0: 伸直
            0.0f     // middle_1: 伸直
        };

        // 剪刀 - 食指中指伸展，其他收拢
        gripper_gestures_["scissors"] = {
            0.0f,    // thumb_0
            0.0f,    // thumb_1
            1.0f,    // thumb_2: 弯曲
            -0.2f,   // index_0: 略微弯曲
            -0.2f,   // index_1
            -1.3f,   // middle_0: 收拢
            -1.5f    // middle_1: 收拢
        };
    }

    // ========== 回调函数 ==========
    void armStateCallback(unitree_hg::msg::LowState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(arm_state_mutex_);
        if (!is_arm_ready_) {
            arm_state_ = msg;
            is_arm_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "Robot arm state received! Ready for commands.");
        }
    }

    void handStateCallback(unitree_hg::msg::HandState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(hand_state_mutex_);
        hand_state_ = msg;
    }

    void rpsCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!is_arm_ready_) {
            RCLCPP_WARN(this->get_logger(), "Robot not ready yet, ignoring command");
            return;
        }

        std::string cmd = normalize_cmd(msg->data);
        if (cmd.empty()) {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", msg->data.c_str());
            return;
        }

        // 防抖
        auto now = this->now();
        if (cmd == last_cmd_ && (now - last_cmd_time_).seconds() < 0.5) {
            RCLCPP_WARN(this->get_logger(), "Debounced duplicate: %s", cmd.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Executing RPS command: %s", cmd.c_str());

        // 执行动作序列
        executeRPSAction(cmd);

        last_cmd_ = cmd;
        last_cmd_time_ = now;
    }

    // ========== 动作执行 ==========
    void executeRPSAction(const std::string& gesture)
    {
        // 1. 移动手臂到游戏位置（如果还未到位）
        if (!isArmAtGamePosition()) {
            RCLCPP_INFO(this->get_logger(), "Moving arm to game position...");
            moveArmToPosition(arm_game_pos_, 2.0f);  // 2秒到达
        }

        // 2. 执行夹爪手势
        RCLCPP_INFO(this->get_logger(), "Executing gripper gesture: %s", gesture.c_str());
        executeGripperGesture(gesture);
    }

    bool isArmAtGamePosition()
    {
        // 简单检查：是否已经在游戏位置附近
        for (size_t i = 0; i < 7; ++i) {
            if (std::abs(current_arm_pos_[i] - arm_game_pos_[i]) > 0.1f) {
                return false;
            }
        }
        return true;
    }

    void moveArmToPosition(const std::array<float, 7>& target_pos, float duration)
    {
        int num_steps = static_cast<int>(duration / control_dt_);

        unitree_hg::msg::LowCmd cmd;

        for (int step = 0; step < num_steps; ++step)
        {
            float alpha = static_cast<float>(step) / num_steps;

            // 插值计算当前目标位置
            for (size_t i = 0; i < 7; ++i)
            {
                float pos = current_arm_pos_[i] * (1 - alpha) + target_pos[i] * alpha;

                cmd.motor_cmd.at(left_arm_joints_[i]).set__q(pos);
                cmd.motor_cmd.at(left_arm_joints_[i]).set__dq(0.0f);
                cmd.motor_cmd.at(left_arm_joints_[i]).set__kp(kp_arm_);
                cmd.motor_cmd.at(left_arm_joints_[i]).set__kd(kd_arm_);
                cmd.motor_cmd.at(left_arm_joints_[i]).set__tau(0.0f);
            }

            // 设置权重为1（启用控制）
            cmd.motor_cmd.at(JointIndex::kNotUsedJoint).set__q(1.0f);

            arm_cmd_pub_->publish(cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(control_dt_ * 1000)));
        }

        current_arm_pos_ = target_pos;
    }

    void executeGripperGesture(const std::string& gesture)
    {
        auto it = gripper_gestures_.find(gesture);
        if (it == gripper_gestures_.end()) {
            RCLCPP_ERROR(this->get_logger(), "Unknown gesture: %s", gesture.c_str());
            return;
        }

        const auto& target_pos = it->second;

        unitree_hg::msg::HandCmd msg;
        msg.motor_cmd.resize(MOTOR_MAX);

        for (int i = 0; i < MOTOR_MAX; i++)
        {
            RIS_Mode_t ris_mode;
            ris_mode.id = i;
            ris_mode.status = 0x01;
            ris_mode.timeout = 0;

            uint8_t mode = 0;
            mode |= (ris_mode.id & 0x0F);
            mode |= (ris_mode.status & 0x07) << 4;
            mode |= (ris_mode.timeout & 0x01) << 7;

            msg.motor_cmd[i].set__mode(mode);
            msg.motor_cmd[i].set__q(target_pos[i]);
            msg.motor_cmd[i].set__dq(0.0f);
            msg.motor_cmd[i].set__tau(0.0f);
            msg.motor_cmd[i].set__kp(kp_gripper_);
            msg.motor_cmd[i].set__kd(kd_gripper_);
        }

        hand_cmd_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Gripper gesture sent: %s", gesture.c_str());
    }

    // ========== 成员变量 ==========
    // 发布器和订阅器
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr arm_cmd_pub_;
    rclcpp::Publisher<unitree_hg::msg::HandCmd>::SharedPtr hand_cmd_pub_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr arm_state_sub_;
    rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr hand_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rps_cmd_sub_;

    // 状态变量
    std::mutex arm_state_mutex_;
    std::mutex hand_state_mutex_;
    unitree_hg::msg::LowState::SharedPtr arm_state_;
    unitree_hg::msg::HandState::SharedPtr hand_state_;

    bool is_arm_ready_;
    std::string last_cmd_;
    rclcpp::Time last_cmd_time_;

    // 控制参数
    float control_dt_;
    float kp_arm_, kd_arm_;
    float kp_gripper_, kd_gripper_;

    // 手臂配置
    std::array<JointIndex, 7> left_arm_joints_;
    std::array<float, 7> arm_home_pos_;
    std::array<float, 7> arm_game_pos_;
    std::array<float, 7> current_arm_pos_;

    // 夹爪配置
    std::unordered_map<std::string, std::array<float, 7>> gripper_gestures_;
};

// ========== 主函数 ==========
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RPSArmGripperController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
