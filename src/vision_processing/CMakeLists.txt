cmake_minimum_required(VERSION 3.8)
project(vision_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  vision_processing_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Create resource marker
ament_index_register_resource(vision_processing_plugin
  CONTENT_FILE vision_processing_plugin.xml
)

ament_package()
