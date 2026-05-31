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
    # usbcamera/usbcamera.cpp  
    camera.cpp
    cboard.cpp
    # dm_imu/dm_imu.cpp
    # gimbal/gimbal.cpp
)

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
