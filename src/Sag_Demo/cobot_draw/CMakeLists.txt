cmake_minimum_required(VERSION 3.8)
project(cobot_draw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(draw_gcode src/draw_gcode.cpp)
add_executable(get_current_pose src/get_current_pose.cpp)
add_executable(draw_five_stars src/draw_five_stars.cpp)
add_executable(draw_circle src/draw_circle.cpp)
add_executable(draw_square src/draw_square.cpp)

ament_target_dependencies(draw_gcode
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_msgs
)

ament_target_dependencies(draw_five_stars
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_msgs
)

ament_target_dependencies(draw_circle
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_msgs
)

ament_target_dependencies(draw_square
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_msgs
)


ament_target_dependencies(get_current_pose
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  geometry_msgs
  moveit_msgs
)
install(TARGETS
  draw_gcode
  get_current_pose
  draw_five_stars
  draw_circle
  draw_square
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
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
