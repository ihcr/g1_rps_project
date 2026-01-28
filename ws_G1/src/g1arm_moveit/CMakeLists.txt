cmake_minimum_required(VERSION 3.8)
project(g1arm_moveit)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_go REQUIRED)
# 可选：如果系统里导出了真正的 unitree_api 库，就链接；否则靠源码 crc.cpp
find_package(unitree_api QUIET)

# =========================
# Unitree CRC/Include 定位
# =========================
if(DEFINED ENV{UNITREE_CRC_DIR})
  set(_CRC_DIR "$ENV{UNITREE_CRC_DIR}")
endif()
if(NOT _CRC_DIR)
  if(DEFINED ENV{UNITREE_ROOT})
    set(UNITREE_ROOT "$ENV{UNITREE_ROOT}")
  else()
    set(UNITREE_ROOT "$ENV{HOME}/unitree_ros2")
  endif()
  set(_CAND_COMMON_DIRS
    "${UNITREE_ROOT}/example/src/common"
    "${UNITREE_ROOT}/example/src/src/common"
  )
  foreach(dir ${_CAND_COMMON_DIRS})
    if(EXISTS "${dir}")
      set(_CRC_DIR "${dir}")
      break()
    endif()
  endforeach()
endif()
if(NOT _CRC_DIR)
  message(FATAL_ERROR "Cannot locate Unitree CRC dir. Set UNITREE_ROOT or UNITREE_CRC_DIR.")
endif()

set(_CRC_CANDS
  "${_CRC_DIR}/motor_crc_hg.cpp"
  "${_CRC_DIR}/motor_crc.cpp"
  "${_CRC_DIR}/crc.cpp"
)
set(_CRC_FILE "")
foreach(f ${_CRC_CANDS})
  if(EXISTS "${f}")
    set(_CRC_FILE "${f}")
    break()
  endif()
endforeach()
if(_CRC_FILE STREQUAL "")
  message(FATAL_ERROR "No CRC source found in ${_CRC_DIR}.")
endif()

# 头文件 include 根（…/example/src/include 或 …/example/src/src/include）
set(_UNITREE_INC_CANDIDATES
  "${UNITREE_ROOT}/example/src/include"
  "${UNITREE_ROOT}/example/src/src/include"
)
set(_UNITREE_INC_ROOT "")
foreach(d ${_UNITREE_INC_CANDIDATES})
  if(EXISTS "${d}/common/motor_crc_hg.h"
     OR EXISTS "${d}/common/motor_crc.h"
     OR EXISTS "${d}/common/crc.h")
    set(_UNITREE_INC_ROOT "${d}")
    break()
  endif()
endforeach()
if(_UNITREE_INC_ROOT STREQUAL "")
  message(FATAL_ERROR "Cannot find Unitree include dir near ${UNITREE_ROOT}/example/src(.*/include).")
endif()

# 已安装的 include（很多 utils/* 也可能在这里）
set(_UNITREE_INSTALL_INC      "${UNITREE_ROOT}/cyclonedds_ws/install/include")
set(_UNITREE_INSTALL_INC_GO   "${_UNITREE_INSTALL_INC}/unitree_go")
set(_UNITREE_INSTALL_INC_API  "${_UNITREE_INSTALL_INC}/unitree_api")

# 源码树中的 utils 头 & 源（你的 utils/crc.hpp 在这里）
set(_UNITREE_UTILS_INC        "${UNITREE_ROOT}/unitree_ros2_ws/src/include")

# 可能的 utils 源码目录（探测 crc.cpp）
set(_UTILS_CAND_SRC_DIRS
  "${UNITREE_ROOT}/unitree_ros2_ws/src/src/utils"
  "${UNITREE_ROOT}/unitree_ros2_ws/src/utils"
  "${UNITREE_ROOT}/cyclonedds_ws/src/unitree_api/src/utils"
)
set(_CRC_IMPL "")
foreach(d ${_UTILS_CAND_SRC_DIRS})
  if(EXISTS "${d}/crc.cpp")
    set(_CRC_IMPL "${d}/crc.cpp")
    set(_UNITREE_UTILS_SRC_DIR "${d}")
    break()
  endif()
endforeach()

# 库目录兜底
set(_UNITREE_INSTALL_LIB "${UNITREE_ROOT}/cyclonedds_ws/install/lib")

get_filename_component(_CRC_PARENT "${_CRC_DIR}/.." ABSOLUTE)

message(STATUS "Using Unitree CRC source : ${_CRC_FILE}")
message(STATUS "Using Unitree include    : ${_UNITREE_INC_ROOT}")
message(STATUS "Using Unitree utils inc  : ${_UNITREE_UTILS_INC}")
if(_CRC_IMPL)
  message(STATUS "Using utils CRC impl src : ${_CRC_IMPL}")
else()
  message(WARNING "No utils crc.cpp found; will try to link a library that provides crc32_core.")
endif()
message(STATUS "Unitree install include  : ${_UNITREE_INSTALL_INC}")
message(STATUS "Unitree install lib dir  : ${_UNITREE_INSTALL_LIB}")

add_executable(g1arm_moveit
  src/g1arm_moveit.cpp
  ${_CRC_FILE}
  ${_CRC_IMPL}   # 如果发现了 crc.cpp 就编进去
)

target_include_directories(g1arm_moveit PRIVATE
  "${_UNITREE_INC_ROOT}"
  "${_UNITREE_INC_ROOT}/common"
  "${_CRC_PARENT}"
  "${_CRC_DIR}"
  "${_UNITREE_INSTALL_INC}"
  "${_UNITREE_INSTALL_INC_GO}"
  "${_UNITREE_INSTALL_INC_API}"
  "${_UNITREE_UTILS_INC}"
  "${_UNITREE_UTILS_SRC_DIR}"   # 某些项目会把头也放在 src/utils
)

ament_target_dependencies(g1arm_moveit
  rclcpp rclcpp_action
  control_msgs trajectory_msgs sensor_msgs std_msgs geometry_msgs
  moveit_core moveit_ros_planning_interface
  unitree_hg unitree_go
  # unitree_api 这里不强制；下面“可选链接”
)

# ========== 可选链接 ==========
# 1) 如果 ament 导出了 imported target
if(TARGET unitree_api::unitree_api)
  target_link_libraries(g1arm_moveit PRIVATE unitree_api::unitree_api)
else()
  # 2) 在 install/lib 下寻找普通库名
  find_library(UNITREE_API_LIB NAMES unitree_api PATHS "${_UNITREE_INSTALL_LIB}" NO_DEFAULT_PATH)
  if(UNITREE_API_LIB)
    target_link_libraries(g1arm_moveit PRIVATE "${UNITREE_API_LIB}")
  endif()
endif()
link_directories("${_UNITREE_INSTALL_LIB}")

# 若既没有编进 crc.cpp，又没有链接到提供 crc32_core 的库，则直接报错避免重复踩坑
if(NOT _CRC_IMPL)
  if(NOT TARGET unitree_api::unitree_api AND NOT UNITREE_API_LIB)
    message(FATAL_ERROR
      "Missing crc32_core implementation: not found utils/crc.cpp and no 'unitree_api' library to link.\n"
      "Checked:\n"
      "  - ${_UTILS_CAND_SRC_DIRS}/crc.cpp\n"
      "  - ${_UNITREE_INSTALL_LIB}/libunitree_api.so\n"
      "Fix: ensure one of them exists.")
  endif()
endif()

install(TARGETS g1arm_moveit DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(
  rclcpp rclcpp_action
  control_msgs trajectory_msgs sensor_msgs std_msgs geometry_msgs
  moveit_core moveit_ros_planning_interface
  unitree_hg unitree_go
)

ament_package()

