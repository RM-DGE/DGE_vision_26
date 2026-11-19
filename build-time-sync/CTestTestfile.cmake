# CMake generated Testfile for 
# Source directory: /mnt/c/Users/hhh/Desktop/hhh
# Build directory: /mnt/c/Users/hhh/Desktop/hhh/build-time-sync
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(time_sync_core "/mnt/c/Users/hhh/Desktop/hhh/build-time-sync/time_sync_core_test")
set_tests_properties(time_sync_core PROPERTIES  _BACKTRACE_TRIPLES "/mnt/c/Users/hhh/Desktop/hhh/CMakeLists.txt;83;add_test;/mnt/c/Users/hhh/Desktop/hhh/CMakeLists.txt;0;")
add_test(time_sync_capture "/mnt/c/Users/hhh/Desktop/hhh/build-time-sync/time_sync_capture_test")
set_tests_properties(time_sync_capture PROPERTIES  TIMEOUT "20" _BACKTRACE_TRIPLES "/mnt/c/Users/hhh/Desktop/hhh/CMakeLists.txt;86;add_test;/mnt/c/Users/hhh/Desktop/hhh/CMakeLists.txt;0;")
add_test(time_sync_offline "/usr/bin/python3.10" "/mnt/c/Users/hhh/Desktop/hhh/tests/time_sync_offline_test.py" "/mnt/c/Users/hhh/Desktop/hhh/build-time-sync/calibrate_time_offset" "/mnt/c/Users/hhh/Desktop/hhh/build-time-sync/fixtures")
set_tests_properties(time_sync_offline PROPERTIES  TIMEOUT "240" _BACKTRACE_TRIPLES "/mnt/c/Users/hhh/Desktop/hhh/CMakeLists.txt;93;add_test;/mnt/c/Users/hhh/Desktop/hhh/CMakeLists.txt;0;")
