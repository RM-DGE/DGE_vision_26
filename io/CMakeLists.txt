cmake_minimum_required(VERSION 3.16)

# find_package(yaml-cpp REQUIRED)
# find_package(ament_cmake QUIET)
# find_package(rclcpp QUIET)
# find_package(std_msgs QUIET)
# find_package(rosidl_typesupport_cpp QUIET)
# find_package(sp_msgs QUIET)



# 4. 查找串口库 (libserial)
# 首先尝试使用CMake的find_package
find_package(libserial QUIET)

# 如果找不到，尝试使用pkg-config
if(NOT libserial_FOUND)
    message(STATUS "Using pkg-config to find libserial")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBSERIAL REQUIRED libserial)
    
    # 设置包含目录和链接库
    include_directories(${LIBSERIAL_INCLUDE_DIRS})
    link_directories(${LIBSERIAL_LIBRARY_DIRS})
    
    set(LIBSERIAL_LIBRARIES ${LIBSERIAL_LINK_LIBRARIES})
else()
    set(LIBSERIAL_LIBRARIES libserial::serial)
endif()

# 创建目标 io
add_library(io STATIC 
    hikrobot/hikrobot.cpp
    usbcamera/usbcamera.cpp
    camera.cpp
    cboard.cpp
    Daedalus/daedalus_camera.cpp
    # dm_imu/dm_imu.cpp
    # gimbal/gimbal.cpp
)

# Daedalus 仿真器 IPC 适配层（协议头在 io/Daedalus/，header-only，无额外链接）
target_include_directories(io PUBLIC Daedalus)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

target_link_libraries(io PUBLIC 
    MvCameraControl 
    ${LIBUSB_LIBRARIES} 
    yaml-cpp 
    ${LIBSERIAL_LIBRARIES}
)

# ---------- Foxglove（官方 C++ SDK，原样 vendor 在 foxglove/，用法同 hikrobot/）----------
# 只编 SDK 的 C++ 包装层（12 个源文件）；不要用 src/*.cpp 通配，
# 否则会引入 remote_access.cpp 的未实现 gateway 符号。
add_library(foxglove_cpp STATIC
    foxglove/src/channel.cpp
    foxglove/src/connection_graph.cpp
    foxglove/src/context.cpp
    foxglove/src/error.cpp
    foxglove/src/fetch_asset.cpp
    foxglove/src/foxglove.cpp
    foxglove/src/mcap.cpp
    foxglove/src/messages.cpp
    foxglove/src/parameter.cpp
    foxglove/src/service.cpp
    foxglove/src/websocket.cpp
    foxglove/src/system_info.cpp
)
target_include_directories(foxglove_cpp SYSTEM PUBLIC foxglove/include)
target_link_libraries(foxglove_cpp PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/foxglove/lib/libfoxglove.a"
    pthread dl m
)

target_link_libraries(io PUBLIC foxglove_cpp)

# # mindvision
# target_include_directories(io PUBLIC mindvision/include)
# if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
#   target_link_directories(io PUBLIC mindvision/lib/amd64)
# elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
#   target_link_directories(io PUBLIC mindvision/lib/arm64)
# else()
#   message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
# endif()

# target_link_libraries(io MvCameraControl MVSDK usb-1.0 yaml-cpp serial)

# ################## 检查 ROS 环境 ##################
# find_package(ament_cmake QUIET)
# find_package(rclcpp QUIET)
# find_package(std_msgs QUIET)
# find_package(rosidl_typesupport_cpp QUIET)
# find_package(sp_msgs QUIET)

#  # 如果找不到 ROS 环境，则跳过 ROS 相关的部分
# if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR  NOT rosidl_typesupport_cpp_FOUND OR  NOT std_msgs_FOUND OR NOT sp_msgs_FOUND )
#     message(WARNING "ROS2 not found, skipping ROS2 specific code.")
#  else()
#   message(STATUS "ROS2 environment found, compiling ROS2-related code.")
    
#     # 添加 ROS2 相关的文件
#     target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)

#     # 设置 ROS2 依赖
#     include_directories(${std_msgs_INCLUDE_DIRS})
#     ament_target_dependencies(io rclcpp std_msgs sp_msgs)
#  endif()

# ---------- ROS2 可选桥接层（io_ros2）----------
# 只有 source 了 sp_msgs 所在工作空间、cmake 能找到 rclcpp/sp_msgs 时才编译。
# 这样没有 ROS 环境的机器仍然可以只编译自瞄主程序。
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(sp_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)

if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND sensor_msgs_FOUND
   AND geometry_msgs_FOUND AND sp_msgs_FOUND AND rosidl_typesupport_cpp_FOUND)
  message(STATUS "ROS2 found: building io_ros2 bridge")
  add_library(io_ros2 STATIC
    ros2/mcu_ros_bridge.cpp
  )
  target_include_directories(io_ros2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(io_ros2 io)
  ament_target_dependencies(io_ros2 rclcpp std_msgs sensor_msgs geometry_msgs sp_msgs)
  set(DGE_ROS2_FOUND TRUE PARENT_SCOPE)
else()
  message(STATUS "ROS2/sp_msgs not found: skipping io_ros2 (build ros2_ws/build.sh first)")
  set(DGE_ROS2_FOUND FALSE PARENT_SCOPE)
endif()
