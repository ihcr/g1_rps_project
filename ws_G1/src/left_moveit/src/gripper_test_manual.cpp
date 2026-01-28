/**
 * 手动测试程序 - 用于调试DEX3-1夹爪每个关节
 *
 * 功能：
 * 1. 按数字键0-6选择要控制的关节
 * 2. 按+/-键调整关节角度（步长0.1弧度）
 * 3. 按'r'复位所有关节到中间位置
 * 4. 按'p'打印当前所有关节位置
 * 5. 按'q'退出
 **/

#include <rclcpp/rclcpp.hpp>
#include "unitree_hg/msg/hand_cmd.hpp"
#include "unitree_hg/msg/hand_state.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <mutex>
#include <vector>

constexpr int32_t MOTOR_MAX = 7;

// 左手关节限位
const float maxLimits_left[7] = {1.05, 1.05, 1.75, 0, 0, 0, 0};
const float minLimits_left[7] = {-1.05, -0.724, 0, -1.57, -1.75, -1.57, -1.75};

const std::string HANDCMD_TOPIC = "dex3/left/cmd";
const std::string HANDSTATE_TOPIC = "lf/dex3/left/state";

typedef struct {
    uint8_t id : 4;
    uint8_t status : 3;
    uint8_t timeout : 1;
} RIS_Mode_t;

class GripperTester : public rclcpp::Node
{
public:
    GripperTester() : Node("gripper_tester"), selected_joint_(0)
    {
        handcmd_publisher_ = this->create_publisher<unitree_hg::msg::HandCmd>(HANDCMD_TOPIC, 10);

        handstate_subscriber_ = this->create_subscription<unitree_hg::msg::HandState>(
            HANDSTATE_TOPIC, 10,
            std::bind(&GripperTester::handStateCallback, this, std::placeholders::_1));

        // 初始化到中间位置
        for (int i = 0; i < MOTOR_MAX; i++) {
            current_positions_[i] = (maxLimits_left[i] + minLimits_left[i]) / 2.0;
        }

        // 定时器：检查键盘输入和发送命令
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&GripperTester::timerCallback, this));

        printInstructions();
    }

private:
    void printInstructions()
    {
        std::cout << "\n========== DEX3-1 夹爪手动测试程序 ==========\n";
        std::cout << "控制说明：\n";
        std::cout << "  0-6    : 选择关节 (当前: " << selected_joint_ << ")\n";
        std::cout << "  + / =  : 增加角度 (+0.1 rad)\n";
        std::cout << "  - / _  : 减少角度 (-0.1 rad)\n";
        std::cout << "  [ / ]  : 快速调整 (±0.5 rad)\n";
        std::cout << "  r      : 复位到中间位置\n";
        std::cout << "  p      : 打印当前位置\n";
        std::cout << "  s      : 保存当前位置为预设\n";
        std::cout << "  q      : 退出\n";
        std::cout << "\n关节限位参考：\n";
        for (int i = 0; i < MOTOR_MAX; i++) {
            std::cout << "  关节" << i << ": [" << minLimits_left[i]
                      << ", " << maxLimits_left[i] << "]\n";
        }
        std::cout << "============================================\n\n";
        printCurrentStatus();
    }

    void printCurrentStatus()
    {
        std::cout << "\r当前选择: 关节" << selected_joint_
                  << " | 位置: " << current_positions_[selected_joint_]
                  << " | 范围: [" << minLimits_left[selected_joint_]
                  << ", " << maxLimits_left[selected_joint_] << "]        " << std::flush;
    }

    void handStateCallback(unitree_hg::msg::HandState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        hand_state_ = msg;
    }

    void timerCallback()
    {
        char ch = getNonBlockingInput();

        if (ch == 'q' || ch == 'Q') {
            RCLCPP_INFO(this->get_logger(), "\n退出程序...");
            rclcpp::shutdown();
            return;
        }

        bool need_publish = false;
        bool need_print = false;

        // 选择关节
        if (ch >= '0' && ch <= '6') {
            selected_joint_ = ch - '0';
            std::cout << "\n已选择关节 " << selected_joint_ << "\n";
            need_print = true;
        }
        // 增加角度
        else if (ch == '+' || ch == '=') {
            current_positions_[selected_joint_] += 0.1;
            current_positions_[selected_joint_] = std::min(
                current_positions_[selected_joint_], maxLimits_left[selected_joint_]);
            need_publish = true;
            need_print = true;
        }
        // 减少角度
        else if (ch == '-' || ch == '_') {
            current_positions_[selected_joint_] -= 0.1;
            current_positions_[selected_joint_] = std::max(
                current_positions_[selected_joint_], minLimits_left[selected_joint_]);
            need_publish = true;
            need_print = true;
        }
        // 快速增加
        else if (ch == ']') {
            current_positions_[selected_joint_] += 0.5;
            current_positions_[selected_joint_] = std::min(
                current_positions_[selected_joint_], maxLimits_left[selected_joint_]);
            need_publish = true;
            need_print = true;
        }
        // 快速减少
        else if (ch == '[') {
            current_positions_[selected_joint_] -= 0.5;
            current_positions_[selected_joint_] = std::max(
                current_positions_[selected_joint_], minLimits_left[selected_joint_]);
            need_publish = true;
            need_print = true;
        }
        // 复位
        else if (ch == 'r' || ch == 'R') {
            std::cout << "\n复位到中间位置...\n";
            for (int i = 0; i < MOTOR_MAX; i++) {
                current_positions_[i] = (maxLimits_left[i] + minLimits_left[i]) / 2.0;
            }
            need_publish = true;
            need_print = true;
        }
        // 打印所有位置
        else if (ch == 'p' || ch == 'P') {
            std::cout << "\n\n===== 当前所有关节位置 =====\n";
            for (int i = 0; i < MOTOR_MAX; i++) {
                std::cout << "关节" << i << ": " << current_positions_[i] << "\n";
            }
            std::cout << "==========================\n\n";
        }
        // 保存预设
        else if (ch == 's' || ch == 'S') {
            std::cout << "\n\n===== 保存为C++代码格式 =====\n";
            std::cout << "gesture_positions_[\"custom\"] = {\n";
            for (int i = 0; i < MOTOR_MAX; i++) {
                std::cout << "    " << current_positions_[i] << ",";
                std::cout << "   // joint_" << i;
                if (i == 0) std::cout << " (thumb_0)";
                else if (i == 1) std::cout << " (thumb_1)";
                else if (i == 2) std::cout << " (thumb_2)";
                else if (i == 3) std::cout << " (index_0)";
                else if (i == 4) std::cout << " (index_1)";
                else if (i == 5) std::cout << " (middle_0)";
                else if (i == 6) std::cout << " (middle_1)";
                std::cout << "\n";
            }
            std::cout << "};\n";
            std::cout << "=============================\n\n";
        }

        if (need_publish) {
            publishCommand();
        }
        if (need_print) {
            printCurrentStatus();
        }
    }

    void publishCommand()
    {
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
            msg.motor_cmd[i].set__q(current_positions_[i]);
            msg.motor_cmd[i].set__dq(0.0);
            msg.motor_cmd[i].set__tau(0.0);
            msg.motor_cmd[i].set__kp(1.5);
            msg.motor_cmd[i].set__kd(0.15);
        }

        handcmd_publisher_->publish(msg);
    }

    static char getNonBlockingInput()
    {
        struct termios oldt, newt;
        char ch;
        int oldf;

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

        ch = getchar();

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);

        return ch;
    }

    rclcpp::Publisher<unitree_hg::msg::HandCmd>::SharedPtr handcmd_publisher_;
    rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr handstate_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    int selected_joint_;
    float current_positions_[MOTOR_MAX];

    std::mutex state_mutex_;
    unitree_hg::msg::HandState::SharedPtr hand_state_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperTester>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
