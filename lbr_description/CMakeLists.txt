cmake_minimum_required(VERSION 3.22)
project(lbr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

# install
install(
  DIRECTORY gazebo meshes ros2_control urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.dsv")

if(BUILD_TESTING)
  ament_add_pytest_test(test_urdf ${CMAKE_CURRENT_SOURCE_DIR}/test/test_urdf.py)
endif()

ament_package()
