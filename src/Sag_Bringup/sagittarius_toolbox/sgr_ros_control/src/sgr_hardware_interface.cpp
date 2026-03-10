/*
 *  Copyright (c) 2023, NXROBO Ltd.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  Authors: Litian Zhuang <litian.zhuang@nxrobo.com>
 */

#include <sgr_ros_control/sgr_hardware_interface.hpp>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace sgr_ros_control
{

  CallbackReturn SgrHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    info_ = info;
    nh = std::make_shared<rclcpp::Node>("sgr_hardware_interface");
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(nh);
    group_name = info_.hardware_parameters["group_name"];
    gripper_name = info_.hardware_parameters["gripper_name"];
    std::string js_topic = info_.hardware_parameters["joint_states_topic"];
    pub_group = nh->create_publisher<sagittarius_common_msgs::msg::JointGroupCommand>(
        "commands/joint_group",
        1);
    pub_gripper = nh->create_publisher<sagittarius_common_msgs::msg::JointSingleCommand>(
        "commands/joint_single",
        1);
    sub_joint_states = nh->create_subscription<sensor_msgs::msg::JointState>(
        js_topic,
        1,
        std::bind(&SgrHardwareInterface::joint_state_cb, this, std::placeholders::_1));
    srv_robot_info = nh->create_client<sagittarius_common_msgs::srv::ArmInfo>("get_robot_info");

    using namespace std::chrono_literals;
    // 等待服务开启
    if (!srv_robot_info->wait_for_service(10s)) {
      RCLCPP_FATAL(
        nh->get_logger(),
        "Could not get robot_info service within timeout.");
      throw std::runtime_error("Could not get robot_info service within timeout.");
    }

    // 等待响应
    auto group_info_srv = std::make_shared<sagittarius_common_msgs::srv::ArmInfo::Request>();
    auto group_future = srv_robot_info->async_send_request(group_info_srv);
    executor->spin_until_future_complete(group_future);
    auto group_res = group_future.get();
    num_joints = group_res->num_joints;
    for (int16_t i = 0; i < num_joints; i++)
    {
      joint_state_indices.push_back(i); 
    }  
    joint_state_indices.push_back(num_joints+1);
    joint_positions.resize(num_joints);
    joint_velocities.resize(num_joints);
    joint_efforts.resize(num_joints);
    joint_position_commands.resize(num_joints);
    joint_commands_prev.resize(num_joints);

    // 创建为spin的线程
    update_thread = std::thread(&SgrHardwareInterface::executor_cb, this);

    while (joint_states.position.size() == 0 && rclcpp::ok())
    {
      RCLCPP_INFO_ONCE(nh->get_logger(), "Waiting for joint states...");
    }
    RCLCPP_INFO(nh->get_logger(), "Joint states received.");
    for (size_t i{0}; i < num_joints; i++)
    {
      joint_position_commands.at(i) = joint_states.position.at(joint_state_indices.at(i));
      joint_commands_prev.at(i) = joint_position_commands.at(i);
    }
    joint_commands_prev.resize(num_joints);
    gripper_cmd_prev = joint_states.position.at(joint_state_indices.back()) * 2;
    sagittarius_common_msgs::msg::JointGroupCommand group_cmd;
    group_cmd.name = group_name;
    group_cmd.cmd = group_res->sleep_pos;

    pub_group->publish(group_cmd);
    joint_position_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    joint_positions.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_ERROR(
            nh->get_logger(),
            "Joint '%s' has %d command interfaces found. 1 expected.",
            joint.name.c_str(), static_cast<int>(joint.command_interfaces.size()));
        return CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_ERROR(
            nh->get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
            joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }
    }
    return CallbackReturn::SUCCESS;
  }

  void SgrHardwareInterface::executor_cb()
  {
    executor->spin();
  }

  CallbackReturn SgrHardwareInterface::start()
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn SgrHardwareInterface::stop()
  {
    update_thread.join(); 
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> SgrHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> SgrHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_commands[i]));
    }
    return command_interfaces;
  }

  return_type SgrHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    std::lock_guard<std::mutex> lck(joint_state_mtx_);
    for (size_t i = 0; i < num_joints; i++)
    {
      joint_positions.at(i) = joint_states.position.at(joint_state_indices.at(i));
    }
    return return_type::OK;
  }

  return_type SgrHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    sagittarius_common_msgs::msg::JointGroupCommand group_msg;
    sagittarius_common_msgs::msg::JointSingleCommand gripper_msg;
    group_msg.name = group_name;
    gripper_msg.name = gripper_name;
    gripper_msg.cmd = joint_position_commands.at(num_joints);

    for(size_t i = 0; i < num_joints; i++)
    {
      group_msg.cmd.push_back(joint_position_commands.at(i));
    }
    if(joint_commands_prev != group_msg.cmd)
    {
      pub_group->publish(group_msg);
      joint_commands_prev = group_msg.cmd;
    }
    if((gripper_cmd_prev != gripper_msg.cmd)&&(!std::isnan(gripper_msg.cmd)))
    {
      pub_gripper->publish(gripper_msg);
      gripper_cmd_prev = gripper_msg.cmd;
    }
    return return_type::OK;
  }

  void SgrHardwareInterface::joint_state_cb(const sensor_msgs::msg::JointState &msg)
  {
    std::lock_guard<std::mutex> lck(joint_state_mtx_);
    joint_states = msg;
  }

} // namespace sgr_ros_control

#include "pluginlib/class_list_macros.hpp" // NOLINT
PLUGINLIB_EXPORT_CLASS(
    sgr_ros_control::SgrHardwareInterface,
    hardware_interface::SystemInterface)
