cmake_minimum_required(VERSION 3.8)
project(autonomy_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(autonomy_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(bt_orchestrator
  src/bt_orchestrator.cpp
)

add_executable(bt_debugger
  src/bt_debugger.cpp
)

target_include_directories(bt_orchestrator PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(bt_orchestrator
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

target_link_libraries(bt_debugger
  behaviortree_cpp::behaviortree_cpp
)

ament_target_dependencies(bt_orchestrator
  ament_index_cpp
  autonomy_interfaces
  geometry_msgs
  std_srvs
  tf2_geometry_msgs
  nav_msgs
)

install(TARGETS bt_orchestrator bt_debugger
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

