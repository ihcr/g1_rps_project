cmake_minimum_required(VERSION 3.8)
project(left_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_hg REQUIRED)

add_executable(left_moveit src/left_moveit.cpp)
target_include_directories(left_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(left_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  left_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS left_moveit
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#Bruce
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(grasp_demo src/grasp_demo.cpp)
ament_target_dependencies(grasp_demo
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS grasp_demo
  DESTINATION lib/${PROJECT_NAME})

add_executable(gripper_control_demo src/gripper_control_demo.cpp)
ament_target_dependencies(gripper_control_demo
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS gripper_control_demo
  DESTINATION lib/${PROJECT_NAME})

# Real gripper control (using unitree_hg messages) - LEFT HAND
add_executable(real_gripper_control src/real_gripper_control.cpp)
ament_target_dependencies(real_gripper_control
  rclcpp
  std_msgs
  unitree_hg
)

install(TARGETS real_gripper_control
  DESTINATION lib/${PROJECT_NAME})

# Real gripper control - RIGHT HAND
add_executable(real_gripper_control_right src/real_gripper_control_right.cpp)
ament_target_dependencies(real_gripper_control_right
  rclcpp
  std_msgs
  unitree_hg
)

install(TARGETS real_gripper_control_right
  DESTINATION lib/${PROJECT_NAME})

# Manual gripper testing tool
add_executable(gripper_test_manual src/gripper_test_manual.cpp)
ament_target_dependencies(gripper_test_manual
  rclcpp
  unitree_hg
)

install(TARGETS gripper_test_manual
  DESTINATION lib/${PROJECT_NAME})

# RPS arm and gripper integrated control
add_executable(rps_arm_gripper_control src/rps_arm_gripper_control.cpp)
ament_target_dependencies(rps_arm_gripper_control
  rclcpp
  std_msgs
  unitree_hg
)

install(TARGETS rps_arm_gripper_control
  DESTINATION lib/${PROJECT_NAME})

# Official G1 DEX3 example
add_executable(g1_dex3_example src/g1_dex3_example.cpp)
target_include_directories(g1_dex3_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(g1_dex3_example
  rclcpp
  unitree_hg
)

install(TARGETS g1_dex3_example
  DESTINATION lib/${PROJECT_NAME})

# Test program for middle finger (motor 5 & 6)
add_executable(test_middle_finger src/test_middle_finger.cpp)
ament_target_dependencies(test_middle_finger
  rclcpp
  unitree_hg
)

install(TARGETS test_middle_finger
  DESTINATION lib/${PROJECT_NAME})
#Bruce

ament_package()
