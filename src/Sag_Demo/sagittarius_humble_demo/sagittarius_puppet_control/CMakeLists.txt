cmake_minimum_required(VERSION 3.5)
project(sagittarius_puppet_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sagittarius_common_msgs REQUIRED)
find_package(rosbag2_transport REQUIRED)

# Add executable
add_executable(puppet_control_single_node src/puppet_control_single_node.cpp)
add_executable(ros2bag_record_control src/ros2bag_record_control.cpp)
# add_executable(puppet_control_play scripts/puppet_control_play.py)

set(TARGET_DEPENDENCIES
  rclcpp
  sensor_msgs
  std_msgs
  sagittarius_common_msgs
  rosbag2_transport
)

# Link ament dependencies
ament_target_dependencies(puppet_control_single_node 
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  sagittarius_common_msgs
)

ament_target_dependencies(ros2bag_record_control 
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  sagittarius_common_msgs
)


# Install executable
install(TARGETS puppet_control_single_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ros2bag_record_control
  DESTINATION lib/${PROJECT_NAME})

install(
     DIRECTORY scripts/
     DESTINATION lib/${PROJECT_NAME}
     USE_SOURCE_PERMISSIONS
   )

# Install launch files
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
