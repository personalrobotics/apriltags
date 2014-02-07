cmake_minimum_required(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV)

# Download and build Swatbotics AprilTags library.
include(ExternalProject)
ExternalProject_Add(apriltags-cpp
   PREFIX ${PROJECT_SOURCE_DIR}/build/apriltags-cpp
   GIT_REPOSITORY https://github.com/swatbotics/apriltags-cpp.git
   INSTALL_COMMAND ""
   CMAKE_ARGS "" 
) 

# Compile the tag detector ROS node.
rosbuild_add_executable(detect_apriltags src/detect_apriltags.cpp)
target_link_libraries(detect_apriltags ${PROJECT_SOURCE_DIR}/build/apriltags-cpp/src/apriltags-cpp-build/libapriltags.a)
target_link_libraries(detect_apriltags ${OpenCV_LIBRARIES})

