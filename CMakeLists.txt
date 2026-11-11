cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
# Fingerprint compiled estimator sources independently of uncommitted Git state.
file(GLOB_RECURSE EKF_STAMP_SOURCES CONFIGURE_DEPENDS
  "${PROJECT_SOURCE_DIR}/tasks/auto_aim/*.cpp" "${PROJECT_SOURCE_DIR}/tasks/auto_aim/*.hpp"
  "${PROJECT_SOURCE_DIR}/tools/ekf_*.cpp" "${PROJECT_SOURCE_DIR}/tools/ekf_*.hpp")
set(EKF_STAMP_INPUT "")
foreach(EKF_FILE IN LISTS EKF_STAMP_SOURCES)
  file(SHA256 "${EKF_FILE}" EKF_FILE_HASH)
  string(APPEND EKF_STAMP_INPUT "${EKF_FILE_HASH}")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${EKF_FILE}")
endforeach()
string(SHA256 EKF_SOURCE_HASH "${EKF_STAMP_INPUT}")
add_compile_definitions(HHH_EKF_SOURCE_HASH="${EKF_SOURCE_HASH}")
option(EKF_ONLY "Build v0.60 algorithm checks and replay without hardware dependencies" OFF)
if(EKF_ONLY)
  include(CTest)
  find_package(fmt REQUIRED)
  find_package(spdlog REQUIRED)
  find_package(nlohmann_json REQUIRED)
  add_library(ekf_core STATIC
    tasks/auto_aim/rmcs_ekf/rmcs_ekf.cpp
    tasks/auto_aim/rmcs_ekf/projection.cpp
    tasks/auto_aim/rmcs_ekf/association.cpp
    tasks/auto_aim/rmcs_ekf/recovery.cpp
    tools/logger.cpp
    tools/math_tools.cpp
    tasks/auto_aim/armor.cpp
    tasks/auto_aim/solver.cpp
    tasks/auto_aim/target.cpp
    tasks/auto_aim/tracker.cpp)
  target_include_directories(ekf_core PUBLIC ${PROJECT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(ekf_core PUBLIC Eigen3::Eigen yaml-cpp ${OpenCV_LIBS} fmt::fmt spdlog::spdlog nlohmann_json::nlohmann_json)
  add_executable(ekf_replay tools/ekf_replay.cpp)
  target_link_libraries(ekf_replay PRIVATE ekf_core)
  if(BUILD_TESTING AND EXISTS "${PROJECT_SOURCE_DIR}/tests/v060_algorithm_test.cpp")
    add_executable(v060_algorithm_test tests/v060_algorithm_test.cpp)
    target_link_libraries(v060_algorithm_test PRIVATE ekf_core)
    add_test(NAME v060_algorithm COMMAND v060_algorithm_test)
  endif()
  return()
endif()

include(CTest)
option(TIME_SYNC_ONLY "Build offline time calibration and tests without hardware/detector dependencies" OFF)
# These targets have no detector, camera SDK, serial, or ROS dependency.
find_package(Threads REQUIRED)
add_library(time_sync_core STATIC
  tools/imu_time_buffer.cpp
  tools/time_sync_config.cpp
  tools/time_sync_session.cpp)
target_include_directories(time_sync_core PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(time_sync_core PUBLIC Eigen3::Eigen yaml-cpp Threads::Threads)

add_library(time_offset_calibration STATIC tools/time_offset_calibration.cpp)
target_link_libraries(time_offset_calibration PUBLIC time_sync_core)

add_library(time_sync_recording STATIC tools/time_sync_recorder.cpp)
target_include_directories(time_sync_recording PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(time_sync_recording PUBLIC time_sync_core ${OpenCV_LIBS})

add_executable(calibrate_time_offset calibration/calibrate_time_offset.cpp)
target_include_directories(calibrate_time_offset PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(calibrate_time_offset PRIVATE time_offset_calibration ${OpenCV_LIBS})

if(BUILD_TESTING)
  add_executable(time_sync_core_test tests/time_sync_core_test.cpp)
  target_link_libraries(time_sync_core_test PRIVATE time_offset_calibration)
  add_test(NAME time_sync_core COMMAND time_sync_core_test)
  add_executable(time_sync_capture_test tests/time_sync_capture_test.cpp)
  target_link_libraries(time_sync_capture_test PRIVATE time_sync_recording)
  add_test(NAME time_sync_capture COMMAND time_sync_capture_test)
  set_tests_properties(time_sync_capture PROPERTIES TIMEOUT 20)
  find_package(Python3 QUIET COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    execute_process(COMMAND ${Python3_EXECUTABLE} -c "import cv2, numpy, yaml"
      RESULT_VARIABLE TIME_SYNC_PYTHON_VISION_AVAILABLE OUTPUT_QUIET ERROR_QUIET)
    if(TIME_SYNC_PYTHON_VISION_AVAILABLE EQUAL 0)
      add_test(NAME time_sync_offline
        COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/time_sync_offline_test.py
          $<TARGET_FILE:calibrate_time_offset> ${CMAKE_CURRENT_BINARY_DIR}/fixtures)
      set_tests_properties(time_sync_offline PROPERTIES TIMEOUT 240)
    endif()
  endif()
endif()

if(TIME_SYNC_ONLY)
  return()
endif()

find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

# 全局添加 OpenCV、Eigen 等头文件目录
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
target_link_libraries(io PUBLIC time_sync_core)
if(BUILD_TESTING)
  target_sources(time_sync_capture_test PRIVATE io/cboard.cpp tools/math_tools.cpp tools/logger.cpp)
  target_compile_definitions(time_sync_capture_test PRIVATE TIME_SYNC_IO_TESTS)
  target_link_libraries(time_sync_capture_test PRIVATE fmt::fmt spdlog::spdlog)
endif()
add_subdirectory(tasks/auto_aim)
# add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)

####################src###################
add_executable(standard src/standard.cpp)
add_executable(sentry_DGE src/sentry_DGE.cpp)
# add_executable(mt_standard src/mt_standard.cpp)
# add_executable(standard_mpc src/standard_mpc.cpp)
# add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
# add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
# add_executable(sentry_multithread src/sentry_multithread.cpp)
# add_executable(sentry_debug src/sentry_debug.cpp)


target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
target_link_libraries(sentry_DGE ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)

# ROS2 桥接层暂时不接入 sentry_DGE：当前调试阶段先保证纯自瞄链路。
# 桥接代码保留在 io/ros2/mcu_ros_bridge.*，需要时再重新链接 io_ros2。
if(DGE_ROS2_FOUND)
  add_executable(mcu_ros_delay_test tests/mcu_ros_delay_test.cpp)
  target_link_libraries(mcu_ros_delay_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io io_ros2)
endif()
# target_link_libraries(mt_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
# target_link_libraries(standard_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
# target_link_libraries(standard_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
# target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
# target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
# target_link_libraries(mt_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
# target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io ${CERES_LIBRARIES})
# target_link_libraries(sentry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io ${CERES_LIBRARIES})

##################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(calibrate_validate calibration/calibrate_validate.cpp)
add_executable(capture_time_sync calibration/capture_time_sync.cpp tools/math_tools.cpp tools/logger.cpp)
target_link_libraries(capture_time_sync PRIVATE time_sync_recording ${OpenCV_LIBS} fmt::fmt spdlog::spdlog io)
execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  OUTPUT_VARIABLE TIME_SYNC_SOURCE_HEAD OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT TIME_SYNC_SOURCE_HEAD)
  set(TIME_SYNC_SOURCE_HEAD "unknown")
endif()
target_compile_definitions(capture_time_sync PRIVATE TIME_SYNC_SOURCE_HEAD="${TIME_SYNC_SOURCE_HEAD}")

target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(calibrate_validate ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

##################tests##################
# add_executable(auto_aim_test tests/auto_aim_test.cpp)
# add_executable(auto_buff_test tests/auto_buff_test.cpp)
# add_executable(camera_detect_test tests/camera_detect_test.cpp)
# add_executable(camera_test tests/camera_test.cpp)
# add_executable(camera_thread_test tests/camera_thread_test.cpp)
add_executable(cboard_serial_test tests/cboard_serial_test.cpp)
# add_executable(fire_test tests/fire_test.cpp)
# add_executable(detector_video_test tests/detector_video_test.cpp)
# add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
# add_executable(dm_test tests/dm_test.cpp)
# add_executable(minimum_vision_system tests/minimum_vision_system.cpp)
add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)

# target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
# target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
# target_link_libraries(camera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)
# target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
# target_link_libraries(camera_thread_test ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
target_link_libraries(cboard_serial_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
# target_link_libraries(fire_test ${OpenCV_LIBS} fmt::fmt tools io)
# target_link_libraries(detector_video_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
# target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
# target_link_libraries(dm_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
# target_link_libraries(minimum_vision_system ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(multi_usbcamera_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

# add_executable(gimbal_test tests/gimbal_test.cpp)
# target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# add_executable(planner_test tests/planner_test.cpp)
# target_link_libraries(planner_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# add_executable(planner_test_offline tests/planner_test_offline.cpp)
# target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# add_executable(standard_test/standard_test.cpp)
# target_link_libraries(standard_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# add_executable(standard_mpc_test/standard_mpc_test.cpp)
# target_link_libraries(standard_mpc_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# add_executable(observation_test tests/observation_test.cpp)
# target_link_libraries(observation_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

# # 检查 ROS 环境
# find_package(ament_cmake QUIET)
# find_package(rclcpp QUIET)
# find_package(std_msgs QUIET)
# find_package(rosidl_typesupport_cpp QUIET)
# find_package(sp_msgs QUIET)
# # 只有在找到 ROS 环境时才会添加 sentry 和 publish_test
# if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
#     # 编译 sentry 和 publish_test
#     add_executable(sentry src/sentry.cpp)
#     add_executable(sentry_bp src/sentry_bp.cpp)
#     add_executable(sentry_debug src/sentry_debug.cpp)
#     add_executable(sentry_multithread src/sentry_multithread.cpp)
#     add_executable(publish_test tests/publish_test.cpp)
#     add_executable(subscribe_test tests/subscribe_test.cpp)
#     add_executable(topic_loop_test tests/topic_loop_test.cpp)

#     target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
#     target_link_libraries(sentry_bp ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
#     target_link_libraries(sentry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
#     target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
#     target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
#     target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
#     target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)
# else()
#     message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
# endif()
