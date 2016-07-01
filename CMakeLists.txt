cmake_minimum_required(VERSION 2.8.3)
project(apriltags)

# Set up the ROS Catkin package settings.
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  std_msgs
  sensor_msgs
  image_transport
  roscpp
  cmake_modules
  message_generation
  geometry_msgs
)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

# Import the yaml-cpp libraries.
include(FindPkgConfig)
pkg_check_modules(Yaml REQUIRED yaml-cpp)

add_message_files(DIRECTORY msg FILES
    AprilTagDetection.msg
    AprilTagDetections.msg
)

generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
)

# Set up the ROS Catkin package settings
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge
                 std_msgs
                 sensor_msgs
                 image_transport
                 roscpp
                 geometry_msgs
)

# CGAL requires that -frounding-math be set.
add_definitions(-frounding-math)

# Download the external Swatbotics repository.
include(ExternalProject)
ExternalProject_Add(apriltags_swatbotics_EXTERNAL
    GIT_REPOSITORY https://github.com/personalrobotics/apriltags-cpp.git
    INSTALL_COMMAND ""
    BUILD_COMMAND "make" # I don't know why this works...
    CMAKE_ARGS -DCMAKE_CXX_FLAGS=-frounding-math -DBUILD_SHARED_LIBS:BOOL=ON
)

# Recover project paths for additional settings.
ExternalProject_Get_Property(apriltags_swatbotics_EXTERNAL
  SOURCE_DIR BINARY_DIR INSTALL_DIR)
set(apriltags_swatbotics_INCLUDE_DIRS
  "${SOURCE_DIR}"
)
set(apriltags_swatbotics_LIBRARIES
  "${BINARY_DIR}/libapriltags.so"
)

# Tell CMake that the external project generated a library so we
# can add dependencies here instead of later.
add_library(apriltags_swatbotics UNKNOWN IMPORTED)
set_property(TARGET apriltags_swatbotics
  PROPERTY IMPORTED_LOCATION
  ${apriltags_swatbotics_LIBRARIES}
)
add_dependencies(apriltags_swatbotics apriltags_swatbotics_EXTERNAL)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Yaml_INCLUDE_DIRS}
    ${apriltags_swatbotics_INCLUDE_DIRS}
)

add_executable(apriltags src/apriltags.cpp)
target_link_libraries(apriltags ${catkin_LIBRARIES})
target_link_libraries(apriltags ${Eigen_LIBRARIES})
target_link_libraries(apriltags ${OpenCV_LIBRARIES})
target_link_libraries(apriltags ${Yaml_LIBRARIES})
target_link_libraries(apriltags apriltags_swatbotics)
add_dependencies(apriltags ${PROJECT_NAME}_gencpp)

install(TARGETS apriltags
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)

install(FILES ${apriltags_swatbotics_LIBRARIES}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
