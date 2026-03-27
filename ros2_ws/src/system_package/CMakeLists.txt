cmake_minimum_required(VERSION 3.16)
project(system_package)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration filters)

# Include directories
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

# Build the pointcloud aggregator node
add_executable(pointcloud_aggregator_node src/pointcloud_aggregator_node.cpp)
ament_target_dependencies(pointcloud_aggregator_node rclcpp livox_ros_driver2 sensor_msgs std_srvs pcl_conversions)
target_link_libraries(pointcloud_aggregator_node ${PCL_LIBRARIES})

# Build the system node executable
add_executable(system_node src/system_node.cpp)
ament_target_dependencies(system_node rclcpp sensor_msgs std_srvs pcl_conversions)
target_link_libraries(system_node ${PCL_LIBRARIES})

# Install executables
install(TARGETS
  pointcloud_aggregator_node
  system_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
