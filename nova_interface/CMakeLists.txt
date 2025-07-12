cmake_minimum_required(VERSION 3.8)
project(nova_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)

add_executable(node_subscriber_imu src/subscriber_imu.cpp)
add_executable(node_subscriber_altimeter src/subscriber_altimeter.cpp)

target_link_libraries(node_subscriber_imu PUBLIC 
  rclcpp::rclcpp 
  ${sensor_msgs_TARGETS})

target_link_libraries(node_subscriber_altimeter PUBLIC 
  rclcpp::rclcpp 
  ${ros_gz_interfaces_TARGETS}
)

install(TARGETS
  node_subscriber_imu
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  node_subscriber_altimeter
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
