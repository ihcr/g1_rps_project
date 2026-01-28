#include <memory>
#include <iostream> // 用于标准输入输出
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "right_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("right_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "right_arm");

  // 手动输入目标位姿
  geometry_msgs::msg::Pose target_pose;
  
  std::cout << "=== 请输入目标位置 ===" << std::endl;
  std::cout << "请输入位置 X 坐标: ";
  std::cin >> target_pose.position.x;
  
  std::cout << "请输入位置 Y 坐标: ";
  std::cin >> target_pose.position.y;
  
  std::cout << "请输入位置 Z 坐标: ";
  std::cin >> target_pose.position.z;

  std::cout << "\n=== 请输入目标方向 (四元数) ===" << std::endl;
  std::cout << "请输入方向 X 分量: ";
  std::cin >> target_pose.orientation.x;
  
  std::cout << "请输入方向 Y 分量: ";
  std::cin >> target_pose.orientation.y;
  
  std::cout << "请输入方向 Z 分量: ";
  std::cin >> target_pose.orientation.z;
  
  std::cout << "请输入方向 W 分量: ";
  std::cin >> target_pose.orientation.w;

  // 显示输入的目标位姿
  std::cout << "\n=== 设置的目标位姿 ===" << std::endl;
  std::cout << "位置: [" << target_pose.position.x << ", " 
            << target_pose.position.y << ", " << target_pose.position.z << "]" << std::endl;
  std::cout << "方向: [" << target_pose.orientation.x << ", " 
            << target_pose.orientation.y << ", " << target_pose.orientation.z 
            << ", " << target_pose.orientation.w << "]" << std::endl;
  
  std::cout << "\n开始规划运动..." << std::endl;

  // 设置目标位姿
  move_group_interface.setPoseTarget(target_pose);

  // 创建运动规划
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // 执行规划
  if(success) {
    std::cout << "规划成功，开始执行运动..." << std::endl;
    move_group_interface.execute(plan);
    std::cout << "运动执行完成！" << std::endl;
  } else {
    RCLCPP_ERROR(logger, "规划失败!");
    std::cerr << "运动规划失败，请检查目标位姿是否可达或输入是否有效。" << std::endl;
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}