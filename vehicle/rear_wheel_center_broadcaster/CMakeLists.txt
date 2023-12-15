cmake_minimum_required(VERSION 3.5)
project(rear_wheel_center_broadcaster)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(rear_wheel_center_broadcaster_node SHARED
  src/rear_wheel_center_broadcaster.cpp
)

rclcpp_components_register_node(rear_wheel_center_broadcaster_node
  PLUGIN "rear_wheel_center_broadcaster::RearWheelCenterBroadcaster"
  EXECUTABLE rear_wheel_center_broadcaster
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
