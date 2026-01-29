cmake_minimum_required(VERSION 3.8)
project(left_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_hg REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Real gripper control - RIGHT HAND (primary)
add_executable(real_gripper_control_right src/real_gripper_control_right.cpp)
ament_target_dependencies(real_gripper_control_right
  rclcpp
  std_msgs
  unitree_hg
)

install(TARGETS real_gripper_control_right
  DESTINATION lib/${PROJECT_NAME})

# Real gripper control - LEFT HAND (backup version)
add_executable(real_gripper_control_left src/real_gripper_control_left.cpp)
ament_target_dependencies(real_gripper_control_left
  rclcpp
  std_msgs
  unitree_hg
)

install(TARGETS real_gripper_control_left
  DESTINATION lib/${PROJECT_NAME})

ament_package()
