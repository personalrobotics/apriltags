if("master" STREQUAL "")
  message(FATAL_ERROR "Tag for git checkout should not be empty.")
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E remove_directory "/homes/awalsman/ros/local/pr-ros-pkg/trunk/perception_utils/apriltags/build/apriltags-cpp/src/apriltags-cpp"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/homes/awalsman/ros/local/pr-ros-pkg/trunk/perception_utils/apriltags/build/apriltags-cpp/src/apriltags-cpp'")
endif()

execute_process(
  COMMAND "/usr/bin/git" clone "https://github.com/swatbotics/apriltags-cpp.git" "apriltags-cpp"
  WORKING_DIRECTORY "/homes/awalsman/ros/local/pr-ros-pkg/trunk/perception_utils/apriltags/build/apriltags-cpp/src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/swatbotics/apriltags-cpp.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git" checkout master
  WORKING_DIRECTORY "/homes/awalsman/ros/local/pr-ros-pkg/trunk/perception_utils/apriltags/build/apriltags-cpp/src/apriltags-cpp"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'master'")
endif()

execute_process(
  COMMAND "/usr/bin/git" submodule init
  WORKING_DIRECTORY "/homes/awalsman/ros/local/pr-ros-pkg/trunk/perception_utils/apriltags/build/apriltags-cpp/src/apriltags-cpp"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to init submodules in: '/homes/awalsman/ros/local/pr-ros-pkg/trunk/perception_utils/apriltags/build/apriltags-cpp/src/apriltags-cpp'")
endif()

execute_process(
  COMMAND "/usr/bin/git" submodule update --recursive
  WORKING_DIRECTORY "/homes/awalsman/ros/local/pr-ros-pkg/trunk/perception_utils/apriltags/build/apriltags-cpp/src/apriltags-cpp"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/homes/awalsman/ros/local/pr-ros-pkg/trunk/perception_utils/apriltags/build/apriltags-cpp/src/apriltags-cpp'")
endif()

