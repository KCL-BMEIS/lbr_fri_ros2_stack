cmake_minimum_required(VERSION 3.22)
project(lbr_moveit_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(lbr_fri_idl REQUIRED)

# Source files and corresponding targets
set(SRC_FILES
  src/hello_moveit.cpp
  src/spiral_motion.cpp
  src/end_velocity.cpp
  src/position_PID_velocity.cpp
  src/Incremental_PID_velocity.cpp
  src/iopublisher.cpp
)

foreach(SRC_FILE ${SRC_FILES})
  # Extract file name without extension for target name
  get_filename_component(TARGET_NAME ${SRC_FILE} NAME_WE)
  add_executable(${TARGET_NAME} ${SRC_FILE})
  ament_target_dependencies(${TARGET_NAME}
    geometry_msgs
    rclcpp
    sensor_msgs
    moveit_ros_planning_interface
    moveit_msgs
    sensor_msgs
    std_msgs
    tf2
    moveit_core
    moveit_ros_planning
    lbr_fri_idl
  )
  install(TARGETS ${TARGET_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
