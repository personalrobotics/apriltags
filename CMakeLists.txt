cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

find_package( OpenCV REQUIRED )
find_package( Eigen REQUIRED )
add_definitions(-frounding-math)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include(ExternalProject)
ExternalProject_Add(apriltags
    PREFIX ${PROJECT_SOURCE_DIR}/build/apriltags
    GIT_REPOSITORY file:////homes/pkv/ros/local/apriltags-cpp
    INSTALL_COMMAND ""
    CMAKE_ARGS -DCMAKE_CXX_FLAGS=-frounding-math
)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(${PROJECT_SOURCE_DIR}/build/apriltags/src/apriltags/src/)
include_directories(${PROJECT_SOURCE_DIR}/build/apriltags/src/apriltags)
include_directories(${EIGEN_INCLUDE_DIR} )
include_directories(include)

rosbuild_add_executable(apriltags src/apriltags.cpp)
#target_link_libraries(apriltags ${PROJECT_SOURCE_DIR}/build/apriltags/src/apriltags-build/libapriltags.so)
#target_link_libraries(apriltags /homes/awalsman/ros/local/pr-ros-pkg/trunk/perception_utils/apriltags-cpp/build/libapriltags.so)
target_link_libraries(apriltags ${PROJECT_SOURCE_DIR}/build/apriltags/src/apriltags-build/libapriltags.so)
target_link_libraries(apriltags ${OpenCV_LIBS})
target_link_libraries(apriltags ${EIGEN_LIBS})
#target_link_libraries(apriltags ${YAMLCPP_LIBRARY})
target_link_libraries(apriltags yaml-cpp)

