cmake_minimum_required(VERSION 3.8)
project(px4_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# examples/offboard/offboard_control_pos  (uses px4_ros2_cpp interface library)
add_executable(offboard_control_pos src/offboard_control_pos.cpp)
target_link_libraries(offboard_control_pos PUBLIC Eigen3::Eigen px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp ${geometry_msgs_TARGETS})
install(TARGETS offboard_control_pos DESTINATION lib/${PROJECT_NAME})

# Tobias' hjemmebyggede velocity til drone
add_executable(velocityToDrone src/velocityToDrone.cpp)
target_link_libraries(velocityToDrone PUBLIC Eigen3::Eigen px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp ${geometry_msgs_TARGETS})
install(TARGETS velocityToDrone DESTINATION lib/${PROJECT_NAME})

ament_package()
