# Example CMakeLists.txt Configuration for ROS2 Jazzy Testing
# 
# This file demonstrates how to add tests to a ROS2 package's CMakeLists.txt
# following ROS2 Jazzy best practices.
#
# Copy relevant sections to your package's CMakeLists.txt

cmake_minimum_required(VERSION 3.8)
project(your_package_name)

# ... your existing package configuration ...

# ============================================================================
# Testing Configuration
# ============================================================================

# Find testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ros REQUIRED)
  
  # Option 1: Add pytest test (for unit/integration tests)
  # Use this for testing Python code logic
  ament_add_pytest_test(
    test_your_module
    test/test_your_module.py
    APPEND_ENV PYTHONPATH="${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 60
  )
  
  # Option 2: Add launch_testing test (for launch file tests)
  # Use this for testing launch files and system integration
  launch_testing_add_pytest_test(
    test_launch_file
    test/test_launch_file.py
    ARGS ["--ros-args", "--log-level", "info"]
    TIMEOUT 120
  )
  
  # Option 3: Add gtest for C++ tests
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_your_cpp_module
      test/test_your_cpp_module.cpp
    )
    if(TARGET test_your_cpp_module)
      target_include_directories(test_your_cpp_module PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
      )
      target_link_libraries(test_your_cpp_module
        your_library
        ${rclcpp_LIBRARIES}
      )
      ament_target_dependencies(test_your_cpp_module
        rclcpp
        # Add other dependencies as needed
      )
    endif()
  endif()
  
  # Run linting
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
