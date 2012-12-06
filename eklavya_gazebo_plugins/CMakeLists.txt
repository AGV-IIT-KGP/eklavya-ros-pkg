cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

rosbuild_add_library(diffdrive_plugin src/diffdrive_plugin.cpp)
rosbuild_link_boost(diffdrive_plugin thread)

rosbuild_add_boost_directories()

rosbuild_add_library(eklavya_drive_plugin src/eklavya_drive_plugin.cpp)
rosbuild_link_boost(eklavya_drive_plugin thread)

rosbuild_add_library(eklavya_simple_drive_plugin src/eklavya_simple_drive_plugin.cpp)
rosbuild_link_boost(eklavya_simple_drive_plugin thread)
