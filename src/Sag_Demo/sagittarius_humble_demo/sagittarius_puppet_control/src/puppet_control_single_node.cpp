#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sagittarius_common_msgs/srv/arm_info.hpp>
#include <sagittarius_common_msgs/msg/arm_rad_control.hpp>
#include <mutex>
#include <string>

class SagittariusPuppetControlSingle : public rclcpp::Node
{
public:
  SagittariusPuppetControlSingle()
  : Node("sagittarius_puppet_control_single"), loop_rate_(std::make_shared<rclcpp::Rate>(100))
  {
    // 初始化参数
    joint_states_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 100, std::bind(&SagittariusPuppetControlSingle::joint_state_cb, this, std::placeholders::_1));
    robot_info_client_ = this->create_client<sagittarius_common_msgs::srv::ArmInfo>("get_robot_info");
    torque_control_publisher_ = this->create_publisher<std_msgs::msg::String>("control_torque", 1);
    joint_commands_publisher_ = this->create_publisher<sagittarius_common_msgs::msg::ArmRadControl>("joint/commands", 100);
    gripper_command_publisher_ = this->create_publisher<std_msgs::msg::Float64>("gripper/command", 100);

  }

  void joint_state_cb(const sensor_msgs::msg::JointState &msg)
  {
    joint_states = msg;
    // 输出关节状态信息
    // RCLCPP_INFO(this->get_logger(), "Joint states: %f, %f, %f, %f, %f, %f",
    //             msg.position.at(0), msg.position.at(1), msg.position.at(2),
    //             msg.position.at(3), msg.position.at(4), msg.position.at(5));
  }

  void run()
  {
    auto start_time = this->now();  // 记录开始时间

    // 1.等待直到有订阅者订阅关节命令，并且关节状态不为空
    while ((joint_commands_publisher_->get_subscription_count() < 1 || joint_states.position.empty()) && rclcpp::ok())
    {
      if ((this->now() - start_time).seconds() > 10) // 检查是否等待超时（10秒）
      {
        RCLCPP_ERROR(this->get_logger(), "等待订阅或关节状态超时。"); // 记录错误日志并返回
        return;
      }
      rclcpp::spin_some(this->get_node_base_interface());  // 处理回调函数
      loop_rate_->sleep(); // 按照设定的循环频率休眠
    }

    // 2.发布关闭扭矩控制的消息
    std_msgs::msg::String torque_control_msg_;
    torque_control_msg_.data = "close";
    torque_control_publisher_->publish(torque_control_msg_);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 3. 获取机器人信息，提取具体的关节数量
    auto robot_info_request = std::make_shared<sagittarius_common_msgs::srv::ArmInfo::Request>(); // 创建服务请求对象
    // 3.1 确保服务可用
    if (!robot_info_client_->wait_for_service(std::chrono::seconds(1))) { 
        RCLCPP_ERROR(this->get_logger(), "Service not available!");
        return;
    } 
    // 3.2 异步发送服务请求
    auto robot_info_future = robot_info_client_->async_send_request(robot_info_request); 
    std::shared_future response = robot_info_future.share(); 
    if (!response.valid()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send async request.");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Success to send async request.");
    // 3.3 等待服务响应完成
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), response)  
        != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get service response.");
        return;
    }


    size_t cntr = 0;
    // 4. 主循环，持续运行直到节点关闭
    while (rclcpp::ok())
    {
      sagittarius_common_msgs::msg::ArmRadControl pos_msg; // 创建一个 ArmRadControl 消息对象
   
      for (size_t i{0}; i < static_cast<size_t>(response.get()->num_joints); i++) {
        pos_msg.rad.push_back(joint_states.position.at(i));
      }

      // 发布关节角度命令消息
      joint_commands_publisher_->publish(pos_msg);

      // 处理回调函数
      rclcpp::spin_some(this->get_node_base_interface());

      // 按照设定的循环频率休眠
      loop_rate_->sleep();
    }
  }


private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
  rclcpp::Publisher<sagittarius_common_msgs::msg::ArmRadControl>::SharedPtr joint_commands_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_command_publisher_;
  rclcpp::Client<sagittarius_common_msgs::srv::ArmInfo>::SharedPtr robot_info_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr torque_control_publisher_;
  std::shared_ptr<rclcpp::Rate> loop_rate_;
  sensor_msgs::msg::JointState joint_states;
  std::mutex joint_states_mutex_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SagittariusPuppetControlSingle>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
