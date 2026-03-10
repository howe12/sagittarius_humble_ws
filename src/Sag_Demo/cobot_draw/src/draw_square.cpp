#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdexcept>

const double PI = 3.141592653589793;



// 🟢 生成圆形路径点（半径 radius，center_pose 为圆心）
std::vector<std::vector<double>> generateCircleWaypoints(
    const geometry_msgs::msg::Pose& center_pose, double radius, int num_points = 36)
{
    std::vector<std::vector<double>> waypoints;
    for (int i = 0; i < num_points; ++i)
    {
        double angle = 2 * PI * i / num_points;
        geometry_msgs::msg::Pose pose = center_pose;
        double pose_x = pose.position.x+radius * std::cos(angle);
        double pose_y = pose.position.y+radius * std::sin(angle);
        double pose_z = pose.position.z;
        waypoints.push_back({pose_x-0.1, pose_y, pose_z-0.14});
    }
    return waypoints;
}

// 🟢 生成正方形路径点（边长 side_length，center_pose 为中心）
std::vector<std::vector<double>> generateSquareWaypoints(
    const geometry_msgs::msg::Pose& center_pose, double side_length)
{
    std::vector<std::vector<double>> waypoints;
    double half = side_length / 2.0;

    // 顺时针：左下 → 右下 → 右上 → 左上 → 左下（闭合）
    std::vector<std::pair<double, double>> offsets = {
        {-half, -half},
        {+half, -half},
        {+half, +half},
        {-half, +half},
        {-half, -half}
    };

    for (const auto& [dx, dy] : offsets)
    {
        geometry_msgs::msg::Pose pose = center_pose;
        double pose_x = pose.position.x + dx;
        double pose_y = pose.position.y + dy;
        double pose_z = pose.position.z;
        waypoints.push_back({pose_x-0.1, pose_y, pose_z-0.14});
    }
    return waypoints;
}

void draw_gcode(const rclcpp::Node::SharedPtr& node, const std::string &gcode_file_path, const std::vector<double>& init_pose,
                double pen_offset_up, double pen_offset_down, double x_offset, double y_offset)
{
    // 1.配置MoveGroupInterface的运动规划组为sagittarius_arm
    moveit::planning_interface::MoveGroupInterface move_group(node, "sagittarius_arm");
    std::this_thread::sleep_for(std::chrono::seconds(5)); // 添加休眠以确保机器人状态完全更新

    // 2.设置MoveGroupInterface的参数
    move_group.setPlanningTime(10.0);                      // 设置规划时间为10秒     
    move_group.setNumPlanningAttempts(10);                 // 设置规划尝试次数为10次
    move_group.setPlannerId("RRTConnectkConfigDefault");   // 设置规划器为RRTConnectkConfigDefault
    move_group.setMaxVelocityScalingFactor(0.1);           // 设置最大速度缩放因子为0.1
    move_group.setMaxAccelerationScalingFactor(0.1);       // 设置最大加速度缩放因子为0.1
    move_group.setJointValueTarget(init_pose);             // 设置初始关节目标

    // 3.执行初始运动规划
    moveit::planning_interface::MoveGroupInterface::Plan init_plan;
    bool success = (move_group.plan(init_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success){
        move_group.execute(init_plan);}
    else{
        RCLCPP_ERROR(node->get_logger(), "❌ Failed to move to initial pose");
        return;}
    
    // 4.获取当前的位置
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose("sgr532/link_grasping_frame").pose;
    RCLCPP_INFO(node->get_logger(), "📌 Current pose: x=%.2f, y=%.2f, z=%.2f", current_pose.position.x, current_pose.position.y, current_pose.position.z); // 输出当前的位置

    // 5.生成正方形路径点
    std::vector<std::vector<double>> points = generateSquareWaypoints(current_pose, 0.05); // 边长为0.05米
    for (const auto& point : points) { // 输出生成的路径
        RCLCPP_INFO(node->get_logger(), "🤖 Point: x=%.2f, y=%.2f, z=%.2f", point[0], point[1], point[2]);
    }
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (const auto& point : points) // 将路径点转换为geometry_msgs::msg::Pose类型
    {
        geometry_msgs::msg::Pose end_point;
        end_point.position.x = point[0];
        end_point.position.y = point[1];
        end_point.position.z = point[2];
        end_point.orientation = current_pose.orientation; // 保持当前的方向

        waypoints.push_back(end_point);   // 将路径点添加到waypoints向量中
    }

    // 6.生成笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01; // 末端执行器步长
    const double jump_threshold = 0.0; // 跳跃阈值
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);  // 生成笛卡尔路径

    // 7.执行路径规划
    if (fraction > 0.0)
    {
        RCLCPP_INFO(node->get_logger(), "✅ Cartesian path computed successfully. Moving the arm.");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory = trajectory;
        // 尝试执行计划
        auto execute_result = move_group.execute(plan);
        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "✅ Execution succeeded");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "❌ Execution failed with code: %d", execute_result.val);
        }
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "❌ Failed to compute a Cartesian path.");
    }

    geometry_msgs::msg::Pose curr_pose = move_group.getCurrentPose("sgr532/link_grasping_frame").pose;
    double off_x = curr_pose.position.x;
    double off_y = curr_pose.position.y;
    double off_z = curr_pose.position.z;
    // 输出当前的位置
    RCLCPP_ERROR(node->get_logger(), "🤖 Current pose: x=%.2f, y=%.2f, z=%.2f", off_x, off_y, off_z);

}


int main(int argc, char **argv)
{
    // 1.设置节点选项
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true); // 允许从参数覆盖文件中声明参数
    // 2.节点初始化
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("draw_gcode",node_options);

    // 3.获取参数
    std::string gcode_file = node->get_parameter("gcode_file").as_string();
    std::vector<double> init_pose = node->get_parameter("init_pose").as_double_array();
    double pen_offset_up = node->get_parameter("pen_offset_up").as_double();
    double pen_offset_down = node->get_parameter("pen_offset_down").as_double();
    double x_offset = node->get_parameter("x_offset").as_double();
    double y_offset = node->get_parameter("y_offset").as_double();

    // 4.添加节点到执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    // 5.绘制动作
    draw_gcode(node, gcode_file, init_pose, pen_offset_up, pen_offset_down, x_offset, y_offset);

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
