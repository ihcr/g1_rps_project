/**
 * 专门测试关节5和6（middle_0 和 middle_1）的简单测试程序
 **/
#include <rclcpp/rclcpp.hpp>
#include "unitree_hg/msg/hand_cmd.hpp"
#include "unitree_hg/msg/hand_state.hpp"
#include <iostream>
#include <chrono>
#include <thread>

constexpr int32_t MOTOR_MAX = 7;

// 左手关节限位
const float maxLimits_left[7] = {1.05, 1.05, 1.75, 0, 0, 0, 0};
const float minLimits_left[7] = {-1.05, -0.724, 0, -1.57, -1.75, -1.57, -1.75};

typedef struct
{
    uint8_t id : 4;
    uint8_t status : 3;
    uint8_t timeout : 1;
} RIS_Mode_t;

class MiddleFingerTest : public rclcpp::Node
{
public:
    MiddleFingerTest() : Node("middle_finger_test")
    {
        // 发布命令
        handcmd_pub_ = this->create_publisher<unitree_hg::msg::HandCmd>("dex3/left/cmd", 10);

        // 订阅状态
        handstate_sub_ = this->create_subscription<unitree_hg::msg::HandState>(
            "lf/dex3/left/state", 10,
            std::bind(&MiddleFingerTest::stateCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "=== Middle Finger Test (Motor 5 & 6) ===");
        RCLCPP_INFO(this->get_logger(), "Motor 5 limits: %.3f to %.3f", minLimits_left[5], maxLimits_left[5]);
        RCLCPP_INFO(this->get_logger(), "Motor 6 limits: %.3f to %.3f", minLimits_left[6], maxLimits_left[6]);
        RCLCPP_INFO(this->get_logger(), "Starting test in 2 seconds...");

        // 等待2秒让订阅建立
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // 运行测试序列
        runTest();
    }

private:
    void stateCallback(unitree_hg::msg::HandState::SharedPtr msg)
    {
        last_state_ = msg;
    }

    void sendCommand(float pos5, float pos6, const std::string& description)
    {
        RCLCPP_INFO(this->get_logger(), "\n--- %s ---", description.c_str());
        RCLCPP_INFO(this->get_logger(), "Sending: Motor[5] = %.3f, Motor[6] = %.3f", pos5, pos6);

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
            msg.motor_cmd[i].set__tau(0);
            msg.motor_cmd[i].set__kp(1.5);
            msg.motor_cmd[i].set__kd(0.15);

            // 对于关节5和6使用指定的位置，其他关节保持中间位置
            if (i == 5) {
                msg.motor_cmd[i].set__q(pos5);
            } else if (i == 6) {
                msg.motor_cmd[i].set__q(pos6);
            } else {
                float mid = (maxLimits_left[i] + minLimits_left[i]) / 2.0;
                msg.motor_cmd[i].set__q(mid);
            }
            msg.motor_cmd[i].set__dq(0);
        }

        handcmd_pub_->publish(msg);

        // 等待3秒观察运动
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // 打印当前状态
        if (last_state_) {
            RCLCPP_INFO(this->get_logger(), "Current state: Motor[5] = %.3f, Motor[6] = %.3f",
                       last_state_->motor_state[5].q,
                       last_state_->motor_state[6].q);
        }
    }

    void runTest()
    {
        RCLCPP_INFO(this->get_logger(), "\n========== Starting Test Sequence ==========");

        // 测试1: 中间位置
        float mid5 = (maxLimits_left[5] + minLimits_left[5]) / 2.0;
        float mid6 = (maxLimits_left[6] + minLimits_left[6]) / 2.0;
        sendCommand(mid5, mid6, "Test 1: Middle position");

        // 测试2: 最小值
        sendCommand(minLimits_left[5], minLimits_left[6], "Test 2: Min limits");

        // 测试3: 最大值（应该是0）
        sendCommand(maxLimits_left[5], maxLimits_left[6], "Test 3: Max limits (0)");

        // 测试4: 尝试 -0.5
        sendCommand(-0.5, -0.5, "Test 4: -0.5 position");

        // 测试5: 尝试 -1.0
        sendCommand(-1.0, -1.0, "Test 5: -1.0 position");

        // 测试6: 尝试 -1.5
        sendCommand(-1.5, -1.5, "Test 6: -1.5 position");

        // 测试7: 单独测试关节5
        sendCommand(minLimits_left[5], 0, "Test 7: Motor 5 only (min)");

        // 测试8: 单独测试关节6
        sendCommand(0, minLimits_left[6], "Test 8: Motor 6 only (min)");

        RCLCPP_INFO(this->get_logger(), "\n========== Test Complete ==========");
        RCLCPP_INFO(this->get_logger(), "If motors 5 & 6 didn't move, there may be a hardware/firmware issue.");
        RCLCPP_INFO(this->get_logger(), "Check if the fingers are mechanically stuck or disabled.");

        rclcpp::shutdown();
    }

    rclcpp::Publisher<unitree_hg::msg::HandCmd>::SharedPtr handcmd_pub_;
    rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr handstate_sub_;
    unitree_hg::msg::HandState::SharedPtr last_state_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MiddleFingerTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
