#ifndef _SGR_HARDWARE_INTERFACE_HPP_
#define _SGR_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"

#include "sagittarius_common_msgs/msg/joint_group_command.hpp"
#include "sagittarius_common_msgs/msg/joint_single_command.hpp"
#include "sagittarius_common_msgs/srv/robot_info.hpp"
#include "sagittarius_common_msgs/srv/arm_info.hpp"

#include "sensor_msgs/msg/joint_state.hpp"


using hardware_interface::HardwareInfo;
using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using RobotInfo = sagittarius_common_msgs::srv::RobotInfo;
using hardware_interface::HW_IF_POSITION;

namespace sgr_ros_control
{

class SgrHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SgrHardwareInterface)

  /// @brief 初始化 SgrHardwareInterface.
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /// @brief 导出状态接口
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /// @brief 导出命令接口
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /// @brief 开始
  CallbackReturn start();

  /// @brief 结束
  CallbackReturn stop();

  /// @brief 读取数据, 更新 joint_position 
  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  /// @brief 写数据, 发布 /commands 话题
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  /// @brief 返回硬件接口的名称
  /// @return 此硬件接口的名称
  std::string get_name() const final
  {
    return info_.name;
  }

  /// @brief 接收JointState的消息
  /// @param msg joint_states的话题数据
  void joint_state_cb(const sensor_msgs::msg::JointState & msg);

protected:
  // Joint positions populated by the joint_states topic
  std::vector<double> joint_positions;

  // Joint velocities populated by the joint_states topic
  std::vector<double> joint_velocities;

  // Joint efforts populated by the joint_states topic
  std::vector<double> joint_efforts;

  // Position commands for arm group, published to the /commands/joint_group topic
  std::vector<double> joint_position_commands;

  // Position commands for the gripper joint, published to the /commands/joint_single
  float gripper_cmd_prev;

  // Previous update's joint commands
  std::vector<float> joint_commands_prev;

  /// @brief Function allowing the executor to spin in another thread
  void executor_cb();

  // Thread the executor_cb function runs in
  std::thread update_thread;

  // ROS Node for this hardware interface's pubs, subs, and services
  std::shared_ptr<rclcpp::Node> nh;

  // Executor to update this hardware interface's node
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;

  // Publishes arm group commands to the /commands/joint_group topic
  rclcpp::Publisher<sagittarius_common_msgs::msg::JointGroupCommand>::SharedPtr pub_group;

  // Publishes gripper commands to the /commands/joint_single topic
  rclcpp::Publisher<sagittarius_common_msgs::msg::JointSingleCommand>::SharedPtr pub_gripper;

  // Subscribes to the joint_states topic, keeps the joint_* vectors updated
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states;

  // Client for the /get_robot_info service
  rclcpp::Client<sagittarius_common_msgs::srv::ArmInfo>::SharedPtr srv_robot_info;

  // Parameters from the ros2_control tag
  HardwareInfo info_;

  // Name of the arm group, populated by the info_ HardwareInfo
  std::string group_name;

  // Name of the gripper joint, populated by the info_ HardwareInfo
  std::string gripper_name;

  // Helps map joints to their index
  std::vector<int16_t> joint_state_indices;

  // Mutex that locks the joint_state info
  std::mutex joint_state_mtx_;

  // The number of joints in the robot
  size_t num_joints;

  // joint_states message from the joint_states topic
  sensor_msgs::msg::JointState joint_states;
};

}  // namespace sgr_ros_control

#endif  // _SGR_HARDWARE_INTERFACE_HPP_
