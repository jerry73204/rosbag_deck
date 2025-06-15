cmake_minimum_required(VERSION 3.10)
project(rosbag_deck_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosbag_deck_interface REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag_deck_core REQUIRED)

# Add executable
add_executable(rosbag_deck_node
  src/main.cpp
  src/rosbag_deck_node.cpp
)

target_include_directories(rosbag_deck_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(rosbag_deck_node PUBLIC cxx_std_17)

ament_target_dependencies(rosbag_deck_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  builtin_interfaces
  rosbag_deck_interface
  rosbag2_cpp
  rosbag2_storage
)

target_link_libraries(rosbag_deck_node
  rosbag_deck_core::rosbag_deck_core
)

install(TARGETS rosbag_deck_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
