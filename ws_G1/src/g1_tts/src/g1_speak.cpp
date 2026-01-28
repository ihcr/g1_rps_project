/**
 * G1 简化版 TTS 调用程序
 * 基于 g1_audio_client_example.cpp 简化而来
 *
 * 用法: ./g1_speak "You cheated!"
 *
 * 编译方法见 compile_g1_speak.sh
 */

#include "client/g1/g1_audio_client.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace unitree::ros2::g1;

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " \"text to speak\"" << std::endl;
        std::cerr << "Example: " << argv[0] << " \"You cheated!\"" << std::endl;
        return 1;
    }

    std::string text = argv[1];
    std::cout << "[G1 TTS] Text to speak: " << text << std::endl;

    try
    {
        // 初始化 ROS2
        rclcpp::init(argc, argv);

        // 创建 AudioClient（参考示例第237行）
        auto client = std::make_shared<AudioClient>();
        std::cout << "[G1 TTS] AudioClient created" << std::endl;

        // 调用 TtsMaker（参考示例第240行）
        // 参数: text - 要说的文字, 0 - speaker_id
        int32_t ret = client->TtsMaker(text, 0);
        std::cout << "[G1 TTS] TtsMaker returned: " << ret << std::endl;

        if (ret == 0) {
            std::cout << "[G1 TTS] Success! G1 is speaking..." << std::endl;
            // 等待3秒让 G1 完成语音播放
            // 可以根据文字长度调整等待时间
            std::this_thread::sleep_for(std::chrono::seconds(3));
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
