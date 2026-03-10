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
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_real.h>
using namespace std::placeholders;
using namespace std::chrono_literals;

namespace sdk_sagittarius_arm
{
    SagittariusArmReal::SagittariusArmReal(bool & success, const rclcpp::NodeOptions & options) : rclcpp::Node("sdk_sagittarius_arm_real", options)
    {
        sgr_init_parameters();
        if (!sgr_init_driver()) {
            success = false;
            return;
        }
        sgr_init_topics();
        sgr_wait_for_joint_states();
        sgr_init_setup();
        get_param_from_yaml_urdf();
        sgr_init_services();
    }
    SagittariusArmReal::~SagittariusArmReal()
    {
        if(pSDKarm != NULL)
        {
            delete pSDKarm;
        }
        DestroyThread(&mThrcv);

    }
    bool SagittariusArmReal::get_param_from_yaml_urdf()
    {
        int i;
        std::string urdf_string;
        urdf::Model urdfModel;
        YAML::Node joint_configs;
        // parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "/" + robot_model + "/robot_state_publisher");
        parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "/" + robot_name + "/robot_state_publisher");
        while (!parameters_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto parameters = parameters_client_->get_parameters({ "robot_description" });
        for (auto& parameter : parameters)
        {
            if (parameter.get_name() == "robot_description")
            {
                urdf_string = parameter.value_to_string();
                break;
            }
        }
        urdfModel.initString(urdf_string);
        this->declare_parameter<std::string>("joint_configs", "");
        this->get_parameter("joint_configs", filepath_joint_configs);
        try{
            joint_configs = YAML::LoadFile(filepath_joint_configs.c_str());
        } 
        catch (YAML::BadFile & error) {
            RCLCPP_ERROR(LOGGER, "maybe can not read the file:%s",filepath_joint_configs.c_str());
            RCLCPP_FATAL(LOGGER, "YAML Error: '%s'", error.what());
            return false;
        }
        if (joint_configs.IsNull()) 
        {
            RCLCPP_FATAL(LOGGER, "joint config file was not found. Shutting down...");
            return false;
        }        
        YAML::Node sleep_pos = joint_configs["sleep"];
        YAML::Node joint_names = joint_configs["joint_names"];
        arm_info_res.num_joints = joint_configs["joint_num"].as<int8_t>(0);
        for(i=0; i<arm_info_res.num_joints-1; i++)
        {
            arm_info_res.sleep_pos.push_back(sleep_pos[i].as<float>(0.0));
        }
        for(i=0; i<arm_info_res.num_joints; i++)
        {
            arm_info_res.joint_names.push_back(joint_names[i].as<std::string>());
            urdf::JointConstSharedPtr urdf_joint = urdfModel.getJoint(joint_names[i].as<std::string>());
            if (!urdf_joint)
            {
                RCLCPP_INFO(this->get_logger(), "Failed to get joint%s", joint_names[i].as<std::string>());
                return false;
            }
            double upper_limit = urdf_joint->limits->upper;
            double lower_limit = urdf_joint->limits->lower;
            double velocity_limits = urdf_joint->limits->velocity;
            arm_info_res.upper_joint_limits.push_back(upper_limit);
            arm_info_res.lower_joint_limits.push_back(lower_limit);
            arm_info_res.velocity_limits.push_back(velocity_limits);
        }
        arm_info_res.upper_gripper_limit = arm_info_res.upper_joint_limits.at(arm_info_res.num_joints-1);
        arm_info_res.lower_gripper_limit = arm_info_res.lower_joint_limits.at(arm_info_res.num_joints-1);
        return true;
    }



    bool SagittariusArmReal::arm_get_robot_info(
        const std::shared_ptr<rmw_request_id_t> request_header, 
        const std::shared_ptr<sagittarius_common_msgs::srv::ArmInfo::Request> req, 
        const std::shared_ptr<sagittarius_common_msgs::srv::ArmInfo::Response> res)
    {
        (void)request_header;
        *res = arm_info_res;
        RCLCPP_INFO(this->get_logger(), "arm_get_robot_info is called ");
        return true;
    }


    void SagittariusArmReal::sgr_init_setup()
    {
        sleep(1);
        GetAndSetServoVelocity(pSDKarm);
        sleep(1);
        GetAndSetServoAcceleration(pSDKarm);
        sleep(1);
        GetAndSetServoTorque(pSDKarm);
        sleep(1);
    }


    bool SagittariusArmReal::sgr_init_driver()
    {
        int result = sdk_sagittarius_arm::ExitError;
        mThrcv = NULL;
        pSDKarm = NULL;
        pSDKarm = new sdk_sagittarius_arm::CSDarmCommonSerial(serial_port, baudrate, time_limit, exit_free_torque);
        result = pSDKarm->Init();
        if(result != sdk_sagittarius_arm::ExitError)
        {
            StartReceiveSerail();
            return true;
        }
        else 
            return false;
    }


    bool SagittariusArmReal::DestroyThread(boost::thread **th)
    {
        if((*th) != NULL)
        {
            (*th)->interrupt();
            (*th)->join();
            delete (*th);
            (*th) = NULL;
            return true;
        }
        return true;
    }

    void SagittariusArmReal::sgr_init_parameters()
    {
        RCLCPP_INFO(LOGGER, "sgr_init_parameters");
        just_rviz_control = false;
        execute_gripper_traj = false;
        torque_status = true;
        this->declare_parameter<std::string>("serial_port", "/dev/sagittarius");
        this->declare_parameter<std::string>("baudrate", "1000000");
        this->declare_parameter<int>("time_limit", 5);
        this->declare_parameter<std::string>("robot_model", "");
        this->declare_parameter<std::string>("robot_name", "");
        this->declare_parameter<bool>("just_rviz_control", false);
        this->declare_parameter<bool>("exit_free_torque", false);   //程序退出时，是否释放舵机扭矩。

        this->get_parameter("serial_port", serial_port);
        this->get_parameter("baudrate", baudrate);
        this->get_parameter("time_limit", time_limit);
        this->get_parameter("robot_model", robot_model);
        this->get_parameter("robot_name", robot_name);
        this->get_parameter("just_rviz_control", just_rviz_control);
        this->get_parameter("exit_free_torque", exit_free_torque);   
    }

    void SagittariusArmReal::sgr_init_topics()
    {
        sub_command_group = this->create_subscription<sagittarius_common_msgs::msg::JointGroupCommand>(
            "commands/joint_group",
            10,
            std::bind(&SagittariusArmReal::sgr_sub_command_group, this, _1));
        sub_command_single = this->create_subscription<sagittarius_common_msgs::msg::JointSingleCommand>(
            "commands/joint_single",
            10,
            std::bind(&SagittariusArmReal::sgr_sub_command_single, this, _1));
        sub_command_traj = this->create_subscription<sagittarius_common_msgs::msg::JointTrajectoryCommand>(
            "commands/joint_trajectory",
            10,
            std::bind(&SagittariusArmReal::sgr_sub_command_traj, this, _1));
        if(just_rviz_control)
        {
            sub_js = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states",
                10,
                std::bind(&SagittariusArmReal::JointStatesCb, this, _1));
        }
        else
        {
            motionPlan_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
        }
        sub_ct = this->create_subscription<std_msgs::msg::String>(
            "control_torque",
            10,
            std::bind(&SagittariusArmReal::ControlTorque, this, _1));
    }


    void SagittariusArmReal::sgr_init_services()
    {
        srv_get_servo_info = this->create_service<sagittarius_common_msgs::srv::ServoRtInfo>(
            "get_servo_info",
            std::bind(&SagittariusArmReal::arm_get_servo_info, this, _1, _2, _3));

        srv_get_robot_info = this->create_service<sagittarius_common_msgs::srv::ArmInfo>(
            "get_robot_info",
            std::bind(&SagittariusArmReal::arm_get_robot_info, this, _1, _2, _3)); //arm_get_robot_info
    }
    void SagittariusArmReal::sgr_sub_command_group(const sagittarius_common_msgs::msg::JointGroupCommand::SharedPtr msg)
    {
        if(pSDKarm!=NULL)
        {
            angle[0] = msg->cmd[0];
            angle[1] = msg->cmd[1];
            angle[2] = msg->cmd[2];
            angle[3] = msg->cmd[3];
            angle[4] = msg->cmd[4];
            angle[5] = msg->cmd[5];
            if(torque_status)
            {
                pSDKarm->SendArmAllServerCB(angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);			//插补
                //arm_set_gripper_linear_position(cmd_arm.position[6]*2);
                //pSDKarm->SendArmAllServer(angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);
                // RCLCPP_WARN(LOGGER, "[%f,%f,%f,%f,%f,%f]", angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);
            }
        }

    }
    void SagittariusArmReal::sgr_sub_command_single(const sagittarius_common_msgs::msg::JointSingleCommand::SharedPtr msg)
    {
        arm_set_gripper_linear_position(msg->cmd*2.0);

    }
    void SagittariusArmReal::sgr_sub_command_traj(const sagittarius_common_msgs::msg::JointTrajectoryCommand::SharedPtr msg)
    {
        RCLCPP_WARN(LOGGER, "sgr_sub_command_traj");
    }

    void SagittariusArmReal::sgr_wait_for_joint_states()
    {
        rclcpp::Rate r(10);
        while (rclcpp::ok() && !joint_states.name.size() && (!just_rviz_control))
        {
            rclcpp::spin_some(this->get_node_base_interface());
            r.sleep();
        }
    }
    
    void SagittariusArmReal::PublishJointStates(unsigned char *buf)
    {
        sensor_msgs::msg::JointState 		joint_state;
        struct  timeval    tv;
        struct  timezone   tz;
        gettimeofday(&tv,&tz);
        static long long ct,lt;
        joint_state.header.stamp = this->get_clock()->now();
        joint_state.name.resize(8);
        joint_state.position.resize(8);
        joint_state.name[0] = "joint1";
        joint_state.position[0] = (short(buf[0]|buf[1]<<8))/1800.0*PI;
        joint_state.name[1] = "joint2";
        joint_state.position[1] = (short(buf[2]|buf[3]<<8))/1800.0*PI;
        joint_state.name[2] = "joint3";
        joint_state.position[2] = (short(buf[4]|buf[5]<<8))/1800.0*PI;         
        joint_state.name[3] = "joint4";
        joint_state.position[3] = (short(buf[6]|buf[7]<<8))/1800.0*PI;
        joint_state.name[4] = "joint5";
        joint_state.position[4] = (short(buf[8]|buf[9]<<8))/1800.0*PI;
        joint_state.name[5] = "joint6";
        joint_state.position[5] = (short(buf[10]|buf[11]<<8))/1800.0*PI;
        joint_state.name[6] = "joint_gripper_left";
        joint_state.position[6] = -(short(buf[12]|buf[13]<<8))*0.026/900.0;
        joint_state.name[7] = "joint_gripper_right";
        joint_state.position[7] = -(short(buf[12]|buf[13]<<8))*0.026/900.0;
        //	printf("[[[[%f,%f]]]]\n",joint_state.position[6],joint_state.position[7]);
        /*	joint_state.name[6] = "nx07";
                    joint_state.position[6] = 0;*/
        ct = tv.tv_sec*800000+tv.tv_usec;
        if((!just_rviz_control))//ct>=lt)
        {
            //RCLCPP_INFO_ONCE(LOGGER, "ARM->ROS:[%f,%f,%f,%f,%f,%f]", joint_state.position[0],joint_state.position[1],joint_state.position[2],joint_state.position[3],joint_state.position[4],joint_state.position[5]);
            //RCLCPP_INFO(LOGGER, "ARM->ROS:[%f,%f,%f,%f,%f,%f]", joint_state.position[0],joint_state.position[1],joint_state.position[2],joint_state.position[3],joint_state.position[4],joint_state.position[5]);
            joint_states = joint_state;
            motionPlan_pub->publish(joint_state);
            lt = ct+100000;
        }
    }

    void SagittariusArmReal::StartReceiveSerail()
    {
        if(mThrcv == NULL)
            mThrcv = new boost::thread(boost::bind(&SagittariusArmReal::LoopRcv, this));
    }

    void SagittariusArmReal::LoopRcv()
    {
        sleep(2);
        while(rclcpp::ok())
        {

            if(pSDKarm->LoopOnce()==0)
            {

                PublishJointStates(pSDKarm->mFrameBuffer+5);
            }
            else
            {

            }
        }

    }


    bool SagittariusArmReal::GetAndSetServoVelocity(sdk_sagittarius_arm::CSDarmCommon *pt)
    {
        std::string str_arm_vel;
        int arm_vel;
        this->declare_parameter<int>("~arm_velocity", 1000);   //舵机速度。

        this->get_parameter("~arm_velocity", arm_vel);

        RCLCPP_WARN(this->get_logger(),"arm_vel is %d",arm_vel);
        if(pt != NULL)
            pt->SetArmVel(arm_vel);     
       return true;

    }

    bool SagittariusArmReal::GetAndSetServoAcceleration(sdk_sagittarius_arm::CSDarmCommon *pt)
    {
        std::string str_arm_acc;
        int arm_acc;
        this->declare_parameter<int>("~arm_acceleration", 5);   //加速度。

        this->get_parameter("~arm_acceleration", arm_acc);
        RCLCPP_WARN(this->get_logger(),"arm_acceleration is %d",arm_acc);
        if(pt != NULL)
            pt->SetArmAcc(arm_acc);
        return true;
    }

    bool SagittariusArmReal::GetAndSetServoTorque(sdk_sagittarius_arm::CSDarmCommon *pt)
    {
        std::string str_arm_acc;
        int arm_torque[7];

        this->declare_parameter<int>("~servo_torque1", 1000);   
        this->get_parameter("~servo_torque1", arm_torque[0]);
        RCLCPP_INFO(this->get_logger(), "servo_torque1=%d", arm_torque[0]);
        this->declare_parameter<int>("~servo_torque2", 1000);   
        this->get_parameter("~servo_torque2", arm_torque[1]);
        RCLCPP_INFO(this->get_logger(), "servo_torque2=%d", arm_torque[1]);
        this->declare_parameter<int>("~servo_torque3", 1000);   
        this->get_parameter("~servo_torque3", arm_torque[2]);
        RCLCPP_INFO(this->get_logger(), "servo_torque3=%d", arm_torque[2]);
        this->declare_parameter<int>("~servo_torque4", 1000);   
        this->get_parameter("~servo_torque4", arm_torque[3]);
        RCLCPP_INFO(this->get_logger(), "servo_torque4=%d", arm_torque[3]);
        this->declare_parameter<int>("~servo_torque5", 1000);   
        this->get_parameter("~servo_torque5", arm_torque[4]);
        RCLCPP_INFO(this->get_logger(), "servo_torque5=%d", arm_torque[4]);
        this->declare_parameter<int>("~servo_torque6", 1000);   
        this->get_parameter("~servo_torque6", arm_torque[5]);
        RCLCPP_INFO(this->get_logger(), "servo_torque6=%d", arm_torque[5]);
        this->declare_parameter<int>("~servo_torque7", 1000);   
        this->get_parameter("~servo_torque7", arm_torque[6]);
        RCLCPP_INFO(this->get_logger(), "servo_torque7=%d", arm_torque[6]);

        if(pt != NULL)
            pt->SetArmTorque(arm_torque);
        return true;
    }  


    bool SagittariusArmReal::arm_get_servo_info(
        const std::shared_ptr<rmw_request_id_t> request_header, 
        const std::shared_ptr<sagittarius_common_msgs::srv::ServoRtInfo::Request> req, 
        const std::shared_ptr<sagittarius_common_msgs::srv::ServoRtInfo::Response> res)    
    {
        (void)request_header;
        int t_cnt = 500;
        if (req->servo_id>0)
        {
            pSDKarm->servo_state.flag = 0;
            pSDKarm->SendGetServoRealTimeInfo(req->servo_id);
        }    
        else
        {
            RCLCPP_ERROR(this->get_logger(), "the servo id must be more than 0");
            return false;
        }    
        while((pSDKarm->servo_state.flag == 0)&&(t_cnt--))
        {
            usleep(1000*2);
        }
        if(pSDKarm->servo_state.flag)
        {
            res->speed = pSDKarm->servo_state.speed;
            res->voltage = pSDKarm->servo_state.voltage;
            res->current = pSDKarm->servo_state.current;
            res->payload = pSDKarm->servo_state.payload;
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "get the servo state: timeout");
            return false;
        }   


    }


    void SagittariusArmReal::ControlTorque(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "ControlTorque:%s.\n",msg->data.c_str());
        if (msg->data == "open")
        {
            torque_status = true;
            pSDKarm->SendArmLockOrFree(1);
        }
        else
        {
            torque_status = false;
            pSDKarm->SendArmLockOrFree(0);
        }
    }

    void SagittariusArmReal::JointStatesCb(const sensor_msgs::msg::JointState& cmd_arm)
    {
        if(pSDKarm!=NULL)
        {
            angle[0] = cmd_arm.position[0];
            angle[1] = cmd_arm.position[1];
            angle[2] = cmd_arm.position[2];
            angle[3] = cmd_arm.position[3];
            angle[4] = cmd_arm.position[4];
            angle[5] = cmd_arm.position[5];
            if(torque_status)
            {
                pSDKarm->SendArmAllServerCB(angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);			//插补
                arm_set_gripper_linear_position(cmd_arm.position[6]*2);
                //pSDKarm->SendArmAllServer(angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);
                //ROS_INFO("[%f,%f,%f,%f,%f,%f]", angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);
            }
        }

    }


    /// @brief arm_calculate_gripper_degree_position 距离转换成角度
    /// @param dist - 夹爪的距离值
    short SagittariusArmReal::arm_calculate_gripper_degree_position(const float dist)
    {
        double half_dist = dist / 2.0;
        short result = -(3462*half_dist)*10;
        return result;
    }

    /// @brief arm_set_gripper_linear_position 设置夹爪的位置
    /// @param dist - 夹爪的距离值
    void SagittariusArmReal::arm_set_gripper_linear_position(const float dist)
    {
        short g_degree = arm_calculate_gripper_degree_position(dist);
        //printf("degree is %d\n",g_degree);
        arm_set_single_joint_degree_position(g_degree);
    }

    /// @brief arm_set_single_joint_degree_position 发送单个关节的角度。
    /// @param g_degree - 角度值
    void SagittariusArmReal::arm_set_single_joint_degree_position(short g_degree)
    {
        if(torque_status)
        {
            pSDKarm->SendArmEndAction(0,g_degree);
        }
    }

    /// @brief arm_execute_gripper_trajectory    夹爪的轨迹跟踪定时器
    /// @param - TimerEvent 事件定时器参数
    // void SagittariusArmReal::arm_execute_gripper_trajectory(const ros::TimerEvent&)
    // {
    //     static uint8_t cntr = 0;
    //     if (!execute_gripper_traj)
    //     {
    //         if (cntr != 0)
    //         {
    //             RCLCPP_INFO(this->get_logger(), "Gripper Trajectory stopped.");
    //             cntr = 0;
    //         }
    //         return;
    //     }
    //     int traj_size = gripper_tra_msg.points.size();
    //     double time_now = ros::Time::now().toSec() - gripper_start_time;
    //     double time_from_start = gripper_tra_msg.points.at(cntr).time_from_start.toSec();
    //     if (time_now > time_from_start)
    //     {
    //         while (time_now > time_from_start && cntr < (traj_size - 1))
    //         {
    //             cntr++;
    //             time_from_start = gripper_tra_msg.points.at(cntr).time_from_start.toSec();
    //         }
    //         if (cntr < (traj_size - 1))
    //         {
    //             arm_set_gripper_linear_position(gripper_tra_msg.points.at(cntr).positions.at(0)*2.0);
    //         }
    //         else
    //         {
    //             arm_set_gripper_linear_position(gripper_tra_msg.points.at(cntr).positions.at(0)*2.0);
    //             RCLCPP_INFO(this->get_logger(), "Trajectory done being executed.");
    //             execute_gripper_traj = false;
    //             cntr = 0;
    //         }
    //     }
    // }

    /// @brief arm_set_joint_positions - 控制机械臂各个舵机的移动到目标位置
    /// @param joint_positions - 舵机的目标弧度; diff_time - 用时
    void SagittariusArmReal::arm_set_joint_positions(const double joint_positions[], double diff_time)
    {
        if(pSDKarm!=NULL)
        {
            angle[0] = joint_positions[0];
            angle[1] = joint_positions[1];
            angle[2] = joint_positions[2];
            angle[3] = joint_positions[3];
            angle[4] = joint_positions[4];
            angle[5] = joint_positions[5];
            short difftime = diff_time*1000;
            if(torque_status)
            {
                pSDKarm->SendArmAllServerTime(difftime, angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);
            }

        }
    }

    /// @brief arm_write_joint_commands - 处理订阅到的机械臂控制命令。并控制机械臂各个舵机的移动到目标位置
    /// @param msg - 自定义消息。机械臂的位置控制。
    void SagittariusArmReal::arm_write_joint_commands(const sagittarius_common_msgs::msg::ArmRadControl::SharedPtr msg)
    {
        double joint_positions[msg->rad.size()];
        for (size_t i {0}; i < msg->rad.size(); i++)
            joint_positions[i] = msg->rad.at(i);
        arm_set_joint_positions(joint_positions, 0.0);
    }

    /// @brief arm_write_gripper_command - 控制机械臂末端舵机的移动长度
    /// @param msg -
    void SagittariusArmReal::arm_write_gripper_command(const std_msgs::msg::Float64::SharedPtr msg)
    {
        arm_set_gripper_linear_position(msg->data*2.0);
    } 

}







