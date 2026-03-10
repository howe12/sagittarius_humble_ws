
#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestSgrHardwareSystem : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_system_sgr_ =
      R"(
  <ros2_control name="SgrHardwareInterface" type="system">
    <hardware>
      <plugin>sgr_ros_control/SgrHardwareInterface</plugin>
      <param name="loop_hz">10</param>
      <param name="group_name">arm</param>
      <param name="gripper_name">gripper</param>
      <param name="joint_states_topic">joint_states</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="initial_position">1.57</param>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <param name="initial_position">0.7854</param>
    </joint>
  </ros2_control>
)";
  }
  std::string hardware_system_sgr_;
};

TEST_F(TestSgrHardwareSystem, load_hardware_system_sgr_)
{
  // throws due to wait_for_service timeout
  rclcpp::init(0, nullptr);
  auto urdf =
    ros2_control_test_assets::urdf_head + hardware_system_sgr_ + ros2_control_test_assets::urdf_tail;
  EXPECT_THROW(hardware_interface::ResourceManager rm(urdf), std::runtime_error);
}
