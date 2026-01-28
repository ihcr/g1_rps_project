#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gripper_control_demo");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread([&exec]() { exec.spin(); }).detach();

  // MoveGroupInterface for gripper (DEX3-1)
  static const std::string PLANNING_GROUP_GRIPPER = "left_gripper";
  moveit::planning_interface::MoveGroupInterface move_group_gripper(node, PLANNING_GROUP_GRIPPER);

  std::cout << "\n====== DEX3-1 夹爪控制 - 石头剪刀布 ======\n" << std::endl;
  RCLCPP_INFO(node->get_logger(), "Gripper control ready!");

  // 定义三种手势的关节位置
  // 关节顺序: thumb_0, thumb_1, thumb_2, index_0, index_1, middle_0, middle_1

  // 1. 石头 - 三个手指都收拢（使用接近关节限制的最大值）
  std::vector<double> rock_positions = {
    -1.5,   // 上食指第一关节left_hand_thumb_0_joint - 上食指收拢 (max: 1.047)
    -1.5,   // 上食指第二关节left_hand_thumb_1_joint (max: 0.920)
    -1.5,   // 下食指第一关节left_hand_thumb_2_joint (max: 1.745)
    -1.5,  // 下食指第二关节left_hand_index_0_joint - 下食指收拢 (min: -1.571)
    0.0,  // 大拇指第一关节left_hand_index_1_joint (min: -1.745)
    1.0,  // 大拇指第二关节left_hand_middle_0_joint - 大拇指收拢 (min: -1.571)
    1.0   // 大拇指第三关节left_hand_middle_1_joint (min: -1.745)
  };

  // 2. 剪刀 - 大拇指收拢，食指和中指打开
  std::vector<double> scissors_positions = {
    0.0,   // left_hand_thumb_0_joint - 大拇指收拢 (max: 1.047)
    0.0,   // left_hand_thumb_1_joint (max: 0.920)
    0.0,   // left_hand_thumb_2_joint (max: 1.745)
    0.0,    // left_hand_index_0_joint - 食指打开
    0.0,    // left_hand_index_1_joint
    1.0,    // left_hand_middle_0_joint - 中指打开
    1.0     // left_hand_middle_1_joint
  };

  // 3. 布 - 三个手指都打开
  std::vector<double> paper_positions = {
    0.0,    // left_hand_thumb_0_joint - 大拇指打开
    0.0,    // left_hand_thumb_1_joint
    0.0,    // left_hand_thumb_2_joint
    0.0,    // left_hand_index_0_joint - 食指打开
    0.0,    // left_hand_index_1_joint
    0.0,    // left_hand_middle_0_joint - 中指打开
    0.0     // left_hand_middle_1_joint
  };

  // 主循环
  while (rclcpp::ok()) {
    std::cout << "\n请选择手势:" << std::endl;
    std::cout << "1 - 石头（三个手指收拢）" << std::endl;
    std::cout << "2 - 剪刀（大拇指收拢，食指和中指打开）" << std::endl;
    std::cout << "3 - 布（三个手指打开）" << std::endl;
    std::cout << "0 - 退出程序" << std::endl;
    std::cout << "\n请输入数字 (0-3): ";

    int choice;
    std::cin >> choice;

    if (choice == 0) {
      std::cout << "\n退出程序..." << std::endl;
      break;
    }

    std::vector<double> target_positions;
    std::string gesture_name;

    switch (choice) {
      case 1:
        target_positions = rock_positions;
        gesture_name = "石头";
        break;
      case 2:
        target_positions = scissors_positions;
        gesture_name = "剪刀";
        break;
      case 3:
        target_positions = paper_positions;
        gesture_name = "布";
        break;
      default:
        std::cout << "无效输入！请输入 0-3 之间的数字。" << std::endl;
        continue;
    }

    // 执行手势
    std::cout << "\n正在执行手势: " << gesture_name << " ..." << std::endl;
    RCLCPP_INFO(node->get_logger(), "Executing gesture: %s", gesture_name.c_str());

    move_group_gripper.setJointValueTarget(target_positions);
    auto plan = moveit::planning_interface::MoveGroupInterface::Plan();
    bool success = (move_group_gripper.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      move_group_gripper.execute(plan);
      std::cout << "✓ 手势 " << gesture_name << " 执行成功！" << std::endl;
      RCLCPP_INFO(node->get_logger(), "Gesture '%s' executed successfully!", gesture_name.c_str());
    } else {
      std::cout << "✗ 手势 " << gesture_name << " 执行失败！" << std::endl;
      RCLCPP_ERROR(node->get_logger(), "Failed to execute gesture: %s", gesture_name.c_str());
    }
  }

  std::cout << "\n====== 程序结束 ======\n" << std::endl;
  rclcpp::shutdown();
  return 0;
}
