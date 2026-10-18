cmake_minimum_required(VERSION 3.16)

# 多相机感知/决策模块：目前只编译成 OBJECT 库，未链接进任何可执行程序。
# 若要接入，需要在使用方 target_link_libraries(... omniperception ...)，
# 并保证 io/usbcamera 已启用（见 io/CMakeLists.txt）。

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

# OBJECT 库：只生成目标文件，不单独产出 .a/.so
add_library(omniperception OBJECT 
    decider.cpp
    perceptron.cpp
)
target_link_libraries(omniperception openvino::runtime)