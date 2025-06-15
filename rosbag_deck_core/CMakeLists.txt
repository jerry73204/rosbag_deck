cmake_minimum_required(VERSION 3.10)
project(rosbag_deck_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rclcpp REQUIRED)

# Add library
add_library(rosbag_deck_core SHARED
  src/rosbag_deck_core.cpp
  src/rosbag_deck_c.cpp
)

target_include_directories(rosbag_deck_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(rosbag_deck_core PUBLIC cxx_std_17)

# Link rosbag2 libraries
ament_target_dependencies(rosbag_deck_core
  rosbag2_cpp
  rosbag2_storage
  rclcpp
)

# Install library
install(TARGETS rosbag_deck_core
  EXPORT rosbag_deck_core_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Install headers
install(DIRECTORY include/
  DESTINATION include)

# Export targets
install(EXPORT rosbag_deck_core_targets
  FILE rosbag_deck_core_targets.cmake
  NAMESPACE rosbag_deck_core::
  DESTINATION lib/cmake/${PROJECT_NAME})

# Create config files
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/rosbag_deck_core-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/rosbag_deck_core-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/rosbag_deck_core-config-version.cmake
  VERSION 0.1.0
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/rosbag_deck_core-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/rosbag_deck_core-config-version.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(rosbag_deck_core_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosbag2_cpp rosbag2_storage rclcpp)

ament_package()
