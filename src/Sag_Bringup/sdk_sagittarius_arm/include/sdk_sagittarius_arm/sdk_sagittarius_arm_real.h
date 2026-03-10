#ifndef SDK_SAGITTARIUS_ARM_REAL_
#define SDK_SAGITTARIUS_ARM_REAL_


#include <urdf/model.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <memory>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <sensor_msgs/msg/joint_state.h>
#include <sagittarius_common_msgs/msg/arm_rad_control.hpp>
#include <sagittarius_common_msgs/msg/joint_group_command.hpp>
#include <sagittarius_common_msgs/msg/joint_single_command.hpp>
#include <sagittarius_common_msgs/msg/joint_trajectory_command.hpp>


#include <sagittarius_common_msgs/srv/arm_info.hpp>
#include <sagittarius_common_msgs/srv/robot_info.hpp>

#include <sagittarius_common_msgs/srv/servo_rt_info.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
// #include <control_msgs/FollowJointTrajectoryAction.h>
// #include <sdk_sagittarius_arm/ArmRadControl.h>
// #include "sdk_sagittarius_arm/SingleRadControl.h"
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_common_serial.h>
#include <boost/thread.hpp>
// #include <sdk_sagittarius_arm/ArmInfo.h>
// #include <sdk_sagittarius_arm/ServoRtInfo.h>
#include "sdk_sagittarius_arm_common.h"
struct Servo
{
    std::string name;                                                     // 舵机所在的关节名
    uint8_t servo_id;                                                     // 舵机ID。未启用
};


static const auto LOGGER = rclcpp::get_logger("sdk_sagittarius_arm");
namespace sdk_sagittarius_arm
{
    class SagittariusArmReal : public rclcpp::Node
    {
    public:
        explicit SagittariusArmReal(bool & success, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~SagittariusArmReal();
        void sgr_wait_for_joint_states();
        void sgr_init_parameters();
        void sgr_init_topics();
        void sgr_init_services();
        void parse_sgr_yaml();
        bool sgr_init_driver();
        void sgr_init_setup();

        void JointStatesCb(const sensor_msgs::msg::JointState& cmd_arm);
    //    void arm_joint_trajectory_msg_callback(const trajectory_msgs::JointTrajectory &msg);
    //    void arm_joint_trajectory_action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
   //     void arm_gripper_trajectory_action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
    //    void arm_gripper_trajectory_msg_callback(const trajectory_msgs::JointTrajectory &msg);
    //    void arm_execute_joint_trajectory(const ros::TimerEvent&);
        short arm_calculate_gripper_degree_position(const float dist);
        void arm_set_gripper_linear_position(const float dist);
        void arm_set_single_joint_degree_position(short g_degree);
    //    void arm_execute_gripper_trajectory(const ros::TimerEvent&);
        void arm_set_joint_positions(const double joint_positions[], double diff_time);
        void arm_write_joint_commands(const sagittarius_common_msgs::msg::ArmRadControl::SharedPtr msg);

        void arm_write_gripper_command(const std_msgs::msg::Float64::SharedPtr msg);
        void ControlTorque(const std_msgs::msg::String::SharedPtr msg);
        void arm_get_servo_configs(void);
        bool GetAndSetServoAcceleration(sdk_sagittarius_arm::CSDarmCommon *pt);
        bool GetAndSetServoVelocity(sdk_sagittarius_arm::CSDarmCommon *pt);
        bool GetAndSetServoTorque(sdk_sagittarius_arm::CSDarmCommon *pt);
        void PublishJointStates(unsigned char *buf);
        bool arm_get_servo_info(
            const std::shared_ptr<rmw_request_id_t> request_header, 
            const std::shared_ptr<sagittarius_common_msgs::srv::ServoRtInfo::Request> req, 
            const std::shared_ptr<sagittarius_common_msgs::srv::ServoRtInfo::Response> res);
        bool arm_get_robot_info(
            const std::shared_ptr<rmw_request_id_t> request_header, 
            const std::shared_ptr<sagittarius_common_msgs::srv::ArmInfo::Request> req, 
            const std::shared_ptr<sagittarius_common_msgs::srv::ArmInfo::Response> res);
        rclcpp::Subscription<sagittarius_common_msgs::msg::JointGroupCommand>::SharedPtr sub_command_group;

        rclcpp::Subscription<sagittarius_common_msgs::msg::JointSingleCommand>::SharedPtr sub_command_single;         

        rclcpp::Subscription<sagittarius_common_msgs::msg::JointTrajectoryCommand>::SharedPtr sub_command_traj;

        void sgr_sub_command_group(const sagittarius_common_msgs::msg::JointGroupCommand::SharedPtr msg);

        void sgr_sub_command_single(const sagittarius_common_msgs::msg::JointSingleCommand::SharedPtr msg);

        void sgr_sub_command_traj(const sagittarius_common_msgs::msg::JointTrajectoryCommand::SharedPtr msg);
    private:
        std::vector<Servo> arm_joints;
        bool execute_joint_traj;
        bool execute_gripper_traj;
        bool torque_status;
        size_t joint_num_write;
        trajectory_msgs::msg::JointTrajectory jnt_tra_msg,gripper_tra_msg;                                   	
        // actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *joint_action_server;
        // actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *gripper_action_server; 
        sensor_msgs::msg::JointState motionPlan_position;
        sdk_sagittarius_arm::CSDarmCommon *pSDKarm;
        sdk_sagittarius_arm::CSDarmCommon *pTest;

        rclcpp::Service<sagittarius_common_msgs::srv::ArmInfo>::SharedPtr srv_get_robot_info;
        rclcpp::Service<sagittarius_common_msgs::srv::ServoRtInfo>::SharedPtr srv_get_servo_info;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js;
        rclcpp::Subscription<sagittarius_common_msgs::msg::ArmRadControl>::SharedPtr sub_joint_commands;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_gripper_command;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ct;      
 
        float angle[10];
        // ros::Timer tmr_joint_traj;
        // ros::Timer tmr_gripper_traj;
        sensor_msgs::msg::JointState joint_states;
        double joint_start_time;
        bool just_viz_control;
        double gripper_start_time;                                  // 爪子控制的开始时间
        std::vector<double> home_positions;                         
        std::vector<double> sleep_positions;   
        uint8_t *joint_ids_read;                                    
        uint8_t *joint_ids_write;        
        std::vector<Servo> all_joints;                              
        std::string robot_name;                               
        std::string robot_model;      
        std::string serial_port;                    
        std::string baudrate;        
        int time_limit;       
        bool exit_free_torque;     
        bool just_rviz_control;
        std::string filepath_joint_configs;
        int timer_hz;
          bool pub_states;
        std::string js_topic;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motionPlan_pub;
        sagittarius_common_msgs::srv::ArmInfo::Response arm_info_res;

        std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;
        bool get_param_from_yaml_urdf();

        void StartReceiveSerail();
        boost::thread   *mThrcv;
        bool DestroyThread(boost::thread **th);
        void LoopRcv();




    };
} // sdk_sagittarius_arm

#endif // SDK_SAGITTARIUS_ARM_REAL_
