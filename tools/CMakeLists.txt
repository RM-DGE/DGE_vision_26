cmake_minimum_required(VERSION 3.16)

add_library(tools OBJECT
    exiter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    Foxglove/foxglove.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    crc.cpp
)

# tools/Foxglove 用到 io/foxglove 下的官方 SDK 头文件（库由 io 目标链接）
target_include_directories(tools PRIVATE ${CMAKE_SOURCE_DIR}/io/foxglove/include)