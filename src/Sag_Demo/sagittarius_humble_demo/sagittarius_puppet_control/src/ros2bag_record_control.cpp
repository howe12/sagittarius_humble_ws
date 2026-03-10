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
  : Node("sagittarius_puppet_control_single")
  {

    torque_control_publisher_ = this->create_publisher<std_msgs::msg::String>("control_torque", 1);

  }

  void torque_control()
  {
    // 发布关闭扭矩控制的消息
    std_msgs::msg::String torque_control_msg_;
    torque_control_msg_.data = "close";
    torque_control_publisher_->publish(torque_control_msg_);
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }


private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr torque_control_publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SagittariusPuppetControlSingle>();
  node->torque_control();
  rclcpp::shutdown();
  return 0;
}
