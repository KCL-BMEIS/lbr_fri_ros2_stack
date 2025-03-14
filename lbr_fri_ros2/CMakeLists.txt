cmake_minimum_required(VERSION 3.22)
project(lbr_fri_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(FRIClient REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(lbr_fri_idl REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)

# lbr_fri_ros2
add_library(lbr_fri_ros2
  SHARED
    src/interfaces/base_command.cpp
    src/interfaces/position_command.cpp
    src/interfaces/state.cpp
    src/interfaces/torque_command.cpp
    src/interfaces/wrench_command.cpp
    src/app.cpp
    src/async_client.cpp
    src/command_guard.cpp
    src/control.cpp
    src/filters.cpp
    src/ft_estimator.cpp
    src/kinematics.cpp
    src/worker.cpp
)

target_include_directories(lbr_fri_ros2
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lbr_fri_ros2
  Eigen3
  geometry_msgs
  kdl_parser
  lbr_fri_idl
  orocos_kdl_vendor
  rclcpp
  realtime_tools
  std_msgs
)

target_link_libraries(lbr_fri_ros2
  FRIClient::FRIClient
)

ament_export_targets(lbr_fri_ros2_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  eigen3_cmake_module
  Eigen3
  FRIClient
  geometry_msgs
  kdl_parser
  lbr_fri_idl
  orocos_kdl_vendor
  rclcpp
  realtime_tools
  std_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS lbr_fri_ros2
  EXPORT lbr_fri_ros2_export
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # ament_add_gtest(test_command_interfaces test/test_command_interfaces.cpp)
  # target_link_libraries(test_command_interfaces lbr_fri_ros2)

  # # some examples of how to use the interfaces
  # add_executable(test_position_command test/test_position_command.cpp)
  # target_link_libraries(test_position_command lbr_fri_ros2)

  # add_executable(test_torque_command test/test_torque_command.cpp)
  # target_link_libraries(test_torque_command lbr_fri_ros2)

  # add_executable(test_wrench_command test/test_wrench_command.cpp)
  # target_link_libraries(test_wrench_command lbr_fri_ros2)
endif()

ament_package()