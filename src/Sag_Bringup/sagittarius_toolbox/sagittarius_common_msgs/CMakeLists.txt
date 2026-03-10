cmake_minimum_required(VERSION 3.5)
project(sagittarius_common_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

## Generate topics 
set(MSG_FILES
  msg/JointGroupCommand.msg
  msg/JointSingleCommand.msg
  msg/JointTrajectoryCommand.msg
  msg/SingleRadControl.msg
  msg/ArmRadControl.msg
)

## Generate services 
set(SRV_FILES
  srv/RobotInfo.srv
  srv/ArmInfo.srv
  srv/ServoRtInfo.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs trajectory_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
