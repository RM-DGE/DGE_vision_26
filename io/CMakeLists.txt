cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# 仿真模式选项
option(ENABLE_SIM_MODE "Enable simulator mode (ROS2-based)" OFF)

add_subdirectory(serial)

# 创建目标 io
add_library(io STATIC 
    hikrobot/hikrobot.cpp    
    usbcamera/usbcamera.cpp  
    camera.cpp
    cboard.cpp
    gimbal/gimbal.cpp
    ros2/ros2.cpp
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

# # mindvision
# target_include_directories(io PUBLIC mindvision/include)
# if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
#   target_link_directories(io PUBLIC mindvision/lib/amd64)
# elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
#   target_link_directories(io PUBLIC mindvision/lib/arm64)
# else()
#   message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
# endif()

find_library(SYSTEM_SERIAL_LIB serial PATHS /usr/lib /usr/lib/x86_64-linux-gnu)
target_link_libraries(io PUBLIC usb-1.0 yaml-cpp serial ${SYSTEM_SERIAL_LIB} MvCameraControl ${OpenCV_LIBS})

# ROS2 仿真模式
if(ENABLE_SIM_MODE)
    message(STATUS "Simulator mode enabled")
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    
    target_link_libraries(io PUBLIC 
        rclcpp::rclcpp
        sensor_msgs::sensor_msgs
        geometry_msgs::geometry_msgs
        std_msgs::std_msgs
        cv_bridge::cv_bridge
    )
    
    target_compile_definitions(io PUBLIC ENABLE_SIM_MODE)
else()
    message(STATUS "Simulator mode disabled (default: hardware mode)")
endif()
