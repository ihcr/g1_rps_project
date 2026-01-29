/**
 * G1 说 "I Win" 的程序
 * 基于宇树官方 g1_audio_client_example.cpp 简化而来
 *
 * 用法: ./g1_say_i_win
 *
 * 编译方式:
 *   cd ~/ws_G1
 *   colcon build --packages-select g1_tts
 *   source install/setup.bash
 */

#include "client/g1/g1_audio_client.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace unitree::ros2::g1;

int main(int argc, char **argv)
{
    std::cout << "[G1 TTS] Initializing G1 speech: 'I Win'" << std::endl;

    try
    {
        // 初始化 ROS2
        rclcpp::init(argc, argv);

        // 创建 AudioClient
        auto client = std::make_shared<AudioClient>();
        std::cout << "[G1 TTS] AudioClient created successfully" << std::endl;

        // 调用 TtsMaker 让 G1 说 "我赢了"
        // 参数: text - 要说的文字, 0 - speaker_id (默认说话人)
        std::string text = "我赢了";
        int32_t ret = client->TtsMaker(text, 0);
        std::cout << "[G1 TTS] TtsMaker returned: " << ret << std::endl;

        if (ret == 0) {
            std::cout << "[G1 TTS] Success! G1 is saying: 我赢了" << std::endl;
            // 等待2秒让 G1 完成语音播放（"I Win" 比较短）
            std::this_thread::sleep_for(std::chrono::seconds(2));
        } else {
            std::cerr << "[G1 TTS] ERROR: TtsMaker failed with code " << ret << std::endl;
            std::cerr << "[G1 TTS] Make sure G1 robot is connected and audio service is running" << std::endl;
        }

        // 关闭 ROS2
        rclcpp::shutdown();
        return ret;
    }
    catch (const rclcpp::exceptions::RCLError &e)
    {
        std::cerr << "[G1 TTS] RCL Error: " << e.what() << std::endl;
        return 1;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[G1 TTS] Exception: " << e.what() << std::endl;
        return 1;
    }
}
