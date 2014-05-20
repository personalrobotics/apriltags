cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED)
catkin_package(
    INCLUDE_DIRS include/
    LIBRARIES ${PROJECT_NAME}
    DEPENDS OpenCV Eigen yaml-cpp
)

add_definitions(-frounding-math)

# Download the external Swatbotics repository
include(ExternalProject)
ExternalProject_Add(apriltags_swatbotics_EXTERNAL
    PREFIX ${PROJECT_SOURCE_DIR}/build/apriltags
    GIT_REPOSITORY https://github.com/personalrobotics/apriltags-cpp
    INSTALL_COMMAND ""
    CMAKE_ARGS -DCMAKE_CXX_FLAGS=-frounding-math -DBUILD_SHARED_LIBS:BOOL=ON
)

# Recover project paths for additional settings
ExternalProject_Get_Property(apriltags_swatbotics_EXTERNAL
  SOURCE_DIR BINARY_DIR INSTALL_DIR )

# Tell cmake that the external project generated a library so we
# can add dependencies here instead of later
add_library(apriltags_swatbotics UNKNOWN IMPORTED)
set_property(TARGET apriltags_swatbotics
  PROPERTY IMPORTED_LOCATION
  ${BINARY_DIR}/libapriltags.so
  )
add_dependencies(apriltags_swatbotics apriltags_swatbotics_EXTERNAL)
include_directories(${SOURCE_DIR})

include_directories(
    include/${PROJECT_NAME}
    ${Catkin_INCLUDE_DIRS})

add_executable(apriltags src/apriltags.cpp)
target_link_libraries(apriltags ${Catkin_LIBRARIES})
target_link_libraries(apriltags apriltags_swatbotics)

install(TARGETS apriltags
    EXECUTABLE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS apriltags_swatbotics
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
