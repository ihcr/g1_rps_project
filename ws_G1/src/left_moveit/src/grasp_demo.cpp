#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <thread>
#include <map>

using std::chrono::seconds;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("left_arm_grasp_demo");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  // 1) MoveGroupInterface for arm
  static const std::string PLANNING_GROUP_ARM = "left_arm";
  moveit::planning_interface::MoveGroupInterface move_group_arm(node, PLANNING_GROUP_ARM);
  move_group_arm.setPlanningTime(10.0);
  move_group_arm.setMaxVelocityScalingFactor(0.20);
  move_group_arm.setMaxAccelerationScalingFactor(0.20);

  // MoveGroupInterface for gripper (DEX3-1)
  static const std::string PLANNING_GROUP_GRIPPER = "left_gripper";
  moveit::planning_interface::MoveGroupInterface move_group_gripper(node, PLANNING_GROUP_GRIPPER);

  // 末端效应器使用DEX3-1手掌
  const std::string ee_link = "left_hand_palm_link";

  // Planning Scene Interface
  moveit::planning_interface::PlanningSceneInterface psi;

  // 1.3) 清除旧的碰撞对象
  RCLCPP_INFO(node->get_logger(), "Clearing old collision objects from scene...");
  std::vector<std::string> object_ids = psi.getKnownObjectNames();
  if (!object_ids.empty()) {
    psi.removeCollisionObjects(object_ids);
    RCLCPP_INFO(node->get_logger(), "Removed %zu old objects.", object_ids.size());
    std::this_thread::sleep_for(seconds(1));
  }

  // 1.5) 打开夹爪准备抓取
  RCLCPP_INFO(node->get_logger(), "Opening gripper...");
  move_group_gripper.setNamedTarget("open");
  auto gripper_plan = moveit::planning_interface::MoveGroupInterface::Plan();
  bool gripper_ok = (move_group_gripper.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (gripper_ok) {
    move_group_gripper.execute(gripper_plan);
    RCLCPP_INFO(node->get_logger(), "Gripper opened.");
  } else {
    RCLCPP_WARN(node->get_logger(), "Failed to open gripper, continuing anyway...");
  }
  std::this_thread::sleep_for(seconds(1));

  // 2) 添加一个 Box 到场景（位置可改）
  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = "waist_yaw_link";                // 参考帧：根/基座
  box.id = "grasp_box";

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = prim.BOX;
  prim.dimensions = {0.05, 0.05, 0.12};                  // X Y Z 尺寸（米）

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.28;                            // 你在 RViz 里放的 0.28 0 0 附近
  box_pose.position.y = 0.00;
  box_pose.position.z = 0.00 + prim.dimensions[2] / 2.0; // 放在地面上：z=半高
  box_pose.orientation.w = 1.0;

  box.primitives.push_back(prim);
  box.primitive_poses.push_back(box_pose);
  box.operation = box.ADD;

  psi.applyCollisionObject(box);
  RCLCPP_INFO(node->get_logger(), "Added box to the scene.");
  std::this_thread::sleep_for(seconds(1));               // 等场景同步

  // 3) 预抓取：把末端移到盒子前方一点（相对盒子位姿做个偏置）
  geometry_msgs::msg::PoseStamped pre;
  pre.header.frame_id = box.header.frame_id;
  pre.pose = box_pose;
  pre.pose.position.x -= 0.10;                           // 前方 10 cm
  pre.pose.position.z += 0.10;                           // 稍微抬高一点
  pre.pose.orientation.w = 1.0;

  move_group_arm.setPoseReferenceFrame(pre.header.frame_id);
  move_group_arm.setPoseTarget(pre, ee_link);

  auto plan1 = moveit::planning_interface::MoveGroupInterface::Plan();
  bool ok1 = (move_group_arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok1) { RCLCPP_ERROR(node->get_logger(), "Pre-grasp plan failed."); rclcpp::shutdown(); return 1; }
  move_group_arm.execute(plan1);
  RCLCPP_INFO(node->get_logger(), "Moved to pre-grasp position.");

  // 4) 向前靠近：在预抓取基础上再向 +x 靠近 8 cm
  geometry_msgs::msg::PoseStamped grasp = pre;
  grasp.pose.position.x += 0.08;

  move_group_arm.setPoseTarget(grasp, ee_link);
  auto plan2 = moveit::planning_interface::MoveGroupInterface::Plan();
  bool ok2 = (move_group_arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok2) { RCLCPP_ERROR(node->get_logger(), "Approach plan failed."); rclcpp::shutdown(); return 1; }
  move_group_arm.execute(plan2);
  RCLCPP_INFO(node->get_logger(), "Approached object.");

  // 4.5) 闭合夹爪抓取物体
  RCLCPP_INFO(node->get_logger(), "Closing gripper to grasp...");
  move_group_gripper.setNamedTarget("closed");
  auto gripper_plan2 = moveit::planning_interface::MoveGroupInterface::Plan();
  bool gripper_ok2 = (move_group_gripper.plan(gripper_plan2) == moveit::core::MoveItErrorCode::SUCCESS);
  if (gripper_ok2) {
    move_group_gripper.execute(gripper_plan2);
    RCLCPP_INFO(node->get_logger(), "Gripper closed - object grasped!");
  } else {
    RCLCPP_WARN(node->get_logger(), "Failed to close gripper, continuing anyway...");
  }
  std::this_thread::sleep_for(seconds(1));

  // 5) Attach（把盒子"夹住"）
  moveit_msgs::msg::AttachedCollisionObject attach;
  attach.link_name = ee_link;
  attach.object = box;               // 复用同一个 id
  attach.object.operation = attach.object.ADD;
  // touch_links：包含手腕和夹爪的所有link
  attach.touch_links = {
    ee_link,
    "left_wrist_pitch_link",
    "left_wrist_roll_link",
    "left_wrist_yaw_link",
    "left_hand_thumb_0_link",
    "left_hand_thumb_1_link",
    "left_hand_thumb_2_link",
    "left_hand_index_0_link",
    "left_hand_index_1_link",
    "left_hand_middle_0_link",
    "left_hand_middle_1_link"
  };
  psi.applyAttachedCollisionObject(attach);
  RCLCPP_INFO(node->get_logger(), "Attached box to %s.", ee_link.c_str());
  std::this_thread::sleep_for(seconds(1));

  // 6) 提起（沿 +Z 抬起 15 cm）
  geometry_msgs::msg::PoseStamped lift = grasp;
  lift.pose.position.z += 0.15;

  move_group_arm.setPoseTarget(lift, ee_link);
  auto plan3 = moveit::planning_interface::MoveGroupInterface::Plan();
  bool ok3 = (move_group_arm.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok3) { RCLCPP_ERROR(node->get_logger(), "Lift plan failed."); rclcpp::shutdown(); return 1; }
  move_group_arm.execute(plan3);
  RCLCPP_INFO(node->get_logger(), "Lifted object successfully!");

  // 可选：演示完后打开夹爪放下物体
  std::this_thread::sleep_for(seconds(2));
  RCLCPP_INFO(node->get_logger(), "Opening gripper to release object...");
  move_group_gripper.setNamedTarget("open");
  auto gripper_plan3 = moveit::planning_interface::MoveGroupInterface::Plan();
  if (move_group_gripper.plan(gripper_plan3) == moveit::core::MoveItErrorCode::SUCCESS) {
    move_group_gripper.execute(gripper_plan3);
    RCLCPP_INFO(node->get_logger(), "Object released.");
  }

  RCLCPP_INFO(node->get_logger(), "Grasp demo completed!");
  rclcpp::shutdown();
  return 0;
}

