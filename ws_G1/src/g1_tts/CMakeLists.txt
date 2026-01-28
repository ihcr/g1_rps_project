cmake_minimum_required(VERSION 3.8)
project(g1_tts)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)

# Unitree 路径配置
set(UNITREE_ROOT "$ENV{HOME}/unitree_ros2")
set(UNITREE_ROS2_WS "${UNITREE_ROOT}/unitree_ros2_ws")
set(CYCLONEDDS_WS "${UNITREE_ROOT}/cyclonedds_ws")

# 包含目录
include_directories(
  ${UNITREE_ROS2_WS}/src/include
  ${UNITREE_ROS2_WS}/src/third_party/include
  ${UNITREE_ROS2_WS}/install/unitree_ros2/include
  ${CYCLONEDDS_WS}/install/unitree_api/include
  ${CYCLONEDDS_WS}/install/unitree_go/include
)

# 链接目录
link_directories(
  ${UNITREE_ROS2_WS}/install/unitree_ros2/lib
  ${CYCLONEDDS_WS}/install/unitree_api/lib
  ${CYCLONEDDS_WS}/install/unitree_go/lib
)

# 编译 g1_speak 可执行文件
add_executable(g1_speak src/g1_speak.cpp)

ament_target_dependencies(g1_speak
  rclcpp
  unitree_api
)

# 链接 unitree_ros2 库 (包含 AudioClient)
target_link_libraries(g1_speak
  unitree_ros2
)

# 设置 RPATH 以便运行时找到库
set_target_properties(g1_speak PROPERTIES
  INSTALL_RPATH "${UNITREE_ROS2_WS}/install/unitree_ros2/lib;${CYCLONEDDS_WS}/install/unitree_api/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# 安装可执行文件
install(TARGETS
  g1_speak
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
