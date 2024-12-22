cmake_minimum_required(VERSION 3.22)
project(lbr_fri_idl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(lbr_fri_idl
  msg/LBRCommand.msg
  msg/LBRJointPositionCommand.msg
  msg/LBRState.msg
  msg/LBRTorqueCommand.msg
  msg/LBRWrenchCommand.msg
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(FRIClient REQUIRED)

  # link against messages from same package
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  add_library(${PROJECT_NAME}_library INTERFACE)
  target_include_directories(${PROJECT_NAME}_library INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  target_link_libraries(${PROJECT_NAME}_library INTERFACE
    ${cpp_typesupport_target}
  )

  # test commands
  ament_add_gtest(${PROJECT_NAME}_test_command_interfaces
    test/test_command_interfaces.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test_command_interfaces
    ${PROJECT_NAME}_library
    FRIClient::FRIClient
  )
endif()

ament_package()
