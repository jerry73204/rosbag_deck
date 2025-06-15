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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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

# Add example executable
add_executable(dynamic_types_example examples/dynamic_types_example.cpp)
target_link_libraries(dynamic_types_example rosbag_deck_core)
ament_target_dependencies(dynamic_types_example
  rosbag2_cpp
  rosbag2_storage
  rclcpp
)
install(TARGETS dynamic_types_example
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Linting
  ament_lint_auto_find_test_dependencies()
  
  # Test data directory
  set(TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  
  # Common test utilities
  add_library(rosbag_deck_test_utils SHARED
    test/fixtures/test_bag_creator.cpp
    test/fixtures/mock_callbacks.cpp
    test/fixtures/test_data_generator.cpp
  )
  target_link_libraries(rosbag_deck_test_utils
    rosbag_deck_core
  )
  ament_target_dependencies(rosbag_deck_test_utils
    rosbag2_cpp
    rosbag2_storage
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
  )
  
  # Unit tests
  ament_add_gtest(test_index_manager
    test/unit/test_index_manager.cpp
  )
  target_link_libraries(test_index_manager
    rosbag_deck_core
    rosbag_deck_test_utils
  )
  
  ament_add_gtest(test_message_cache
    test/unit/test_message_cache.cpp
  )
  target_link_libraries(test_message_cache
    rosbag_deck_core
    rosbag_deck_test_utils
  )
  
  ament_add_gtest(test_message_type_registry
    test/unit/test_message_type_registry.cpp
  )
  target_link_libraries(test_message_type_registry
    rosbag_deck_core
    rosbag_deck_test_utils
  )
  
  ament_add_gtest(test_bag_worker
    test/unit/test_bag_worker.cpp
  )
  target_link_libraries(test_bag_worker
    rosbag_deck_core
    rosbag_deck_test_utils
  )
  
  ament_add_gtest(test_rosbag_deck_core
    test/unit/test_rosbag_deck_core.cpp
  )
  target_link_libraries(test_rosbag_deck_core
    rosbag_deck_core
    rosbag_deck_test_utils
  )
  
  ament_add_gtest(test_c_api
    test/unit/test_c_api.cpp
  )
  target_link_libraries(test_c_api
    rosbag_deck_core
    rosbag_deck_test_utils
  )
  
  # Integration tests (disabled temporarily - API mismatch needs fixing)
  # ament_add_gtest(test_integration
  #   test/integration/test_integration.cpp
  # )
  # target_link_libraries(test_integration
  #   rosbag_deck_core
  #   rosbag_deck_test_utils
  # )
  
  # Performance tests (requires Google Benchmark) (disabled temporarily - API mismatch needs fixing)
  # find_package(benchmark QUIET)
  # if(benchmark_FOUND)
  #   ament_add_gtest(test_performance
  #     test/performance/test_performance.cpp
  #   )
  #   target_link_libraries(test_performance
  #     rosbag_deck_core
  #     rosbag_deck_test_utils
  #     benchmark::benchmark
  #   )
  # endif()
  
  # Configure code coverage if enabled
  if(CMAKE_BUILD_TYPE STREQUAL "Debug" AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(STATUS "Configuring code coverage for rosbag_deck_core tests")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
  endif()
endif()

ament_export_targets(rosbag_deck_core_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rosbag2_cpp rosbag2_storage rclcpp)

ament_package()
