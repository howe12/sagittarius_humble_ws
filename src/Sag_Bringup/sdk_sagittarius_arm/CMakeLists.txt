cmake_minimum_required(VERSION 3.8)
project(sdk_sagittarius_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sagittarius_common_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS 
  system 
  thread 
  program_options
)
find_package(PkgConfig REQUIRED)


set(TARGET_DEPENDENCIES
  Boost
  ament_index_cpp
  rclcpp
  sensor_msgs
  urdf
  trajectory_msgs
  std_msgs
  moveit_msgs
  sagittarius_common_msgs
  yaml-cpp
)


include_directories(
  include
)

set(sdk_sagittarius_arm_node_SRCS
  src/sdk_sagittarius_arm.cpp
  src/sdk_sagittarius_arm_common_serial.cpp
  src/sdk_sagittarius_arm_common.cpp
  src/sdk_sagittarius_arm_real.cpp
)



add_executable(sdk_sagittarius_arm_node ${sdk_sagittarius_arm_node_SRCS})
ament_target_dependencies(sdk_sagittarius_arm_node ${TARGET_DEPENDENCIES})
target_link_libraries(sdk_sagittarius_arm_node yaml-cpp)
target_include_directories(sdk_sagittarius_arm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(sdk_sagittarius_arm_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17



install(TARGETS sdk_sagittarius_arm_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch cfg rviz DESTINATION share/${PROJECT_NAME})
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
