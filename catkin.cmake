# Set up the ROS Catkin package settings.
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  std_msgs
  sensor_msgs
  image_transport
  roscpp)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

# Set up the ROS Catkin package settings.
catkin_package()

include(FindPkgConfig)
pkg_check_modules(Yaml REQUIRED yaml-cpp)

# CGAL requires that -frounding-math be set.
add_definitions(-frounding-math)

# Download the external Swatbotics repository.
include(ExternalProject)
ExternalProject_Add(apriltags_swatbotics_EXTERNAL
    PREFIX ${PROJECT_SOURCE_DIR}/build/apriltags
    GIT_REPOSITORY https://github.com/personalrobotics/apriltags-cpp
    INSTALL_COMMAND ""
    CMAKE_ARGS -DCMAKE_CXX_FLAGS=-frounding-math -DBUILD_SHARED_LIBS:BOOL=ON
)

# Recover project paths for additional settings.
ExternalProject_Get_Property(apriltags_swatbotics_EXTERNAL
  SOURCE_DIR BINARY_DIR INSTALL_DIR)
set(apriltags_swatbotics_INCLUDE_DIRS
  "${SOURCE_DIR}"
  CACHE INTERNAL "" FORCE
)

# Tell CMake that the external project generated a library so we
# can add dependencies here instead of later.
add_library(apriltags_swatbotics UNKNOWN IMPORTED)
set_property(TARGET apriltags_swatbotics
  PROPERTY IMPORTED_LOCATION
  ${BINARY_DIR}/libapriltags.so)
add_dependencies(apriltags_swatbotics apriltags_swatbotics_EXTERNAL)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Yaml_INCLUDE_DIRS}
    ${apriltags_swatbotics_INCLUDE_DIRS})

add_executable(apriltags src/apriltags.cpp)
target_link_libraries(apriltags ${catkin_LIBRARIES})
target_link_libraries(apriltags ${Eigen_LIBRARIES})
target_link_libraries(apriltags ${OpenCV_LIBRARIES})
target_link_libraries(apriltags ${Yaml_LIBRARIES})
target_link_libraries(apriltags apriltags_swatbotics)

install(TARGETS apriltags
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
