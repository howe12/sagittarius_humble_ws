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


/**
 * @brief 生成五角星外部五个顶点的空间坐标路径
 * 
 * @param center_x 五角星中心点的X坐标
 * @param center_y 五角星中心点的Y坐标
 * @param center_z 五角星中心点的Z坐标（基准高度）
 * @param size 五角星外接圆直径（尺寸缩放因子）
 * @return std::vector<std::vector<double>> 包含五个顶点坐标的二维向量（每个点[x,y,z]）
 */
std::vector<std::vector<double>> generate_star_outer_path(double center_x, double center_y, double center_z, double size) {
    // 存储路径点的容器，每个元素是长度为3的double向量[x, y, z]
    std::vector<std::vector<double>> points;
    
    // 计算五角星外接圆半径（顶点到中心的距离）
    double R = size / 2.0;

    /**
     * 定义五个顶点角度（从正X轴逆时针计算）
     * 角度顺序对应五角星绘制路径：左下 → 顶点 → 右下 → 左上 → 右上
     * 注意：原始角度定义可能根据机械臂坐标系调整过方向
     */
    std::vector<double> angles = {-144.0, 90.0, -72.0, 162.0, 18.0};

    // 遍历每个角度计算坐标
    for (double angle : angles) {
        // 角度转弧度：三角函数需要弧度制输入
        double rad = angle * PI / 180.0;
        
        // 计算基础坐标（二维平面）
        double base_x = center_x + R * cos(rad);
        double base_y = center_y + R * sin(rad);
        
        /**
         * 实际坐标微调（根据机械臂实际工作空间修正）
         *  - x-0.1：X方向偏移补偿（可能适配工具坐标系）
         *  - z-0.14：Z方向下沉量（确保接触绘图表面）
         */
        points.push_back({
            base_x - 0.1,   // 调整后的X坐标
            base_y,         // 原始Y坐标保持不变
            center_z - 0.14 // 调整后的Z坐标（低于中心基准高度）
        });
    }

    return points;
}

void draw_gcode(const rclcpp::Node::SharedPtr& node, const std::string &gcode_file_path, const std::vector<double>& init_pose,
                double pen_offset_up, double pen_offset_down, double x_offset, double y_offset)
{

    // 1. 初始化MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(node, "sagittarius_arm");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 2. 配置MoveGroupInterface
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    // 3. 设置机械臂初始位置
    move_group.setJointValueTarget(init_pose);
    moveit::planning_interface::MoveGroupInterface::Plan init_plan;
    bool success = (move_group.plan(init_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.execute(init_plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to initial pose");
        return;
    }

    // 4. 获取当前机械臂的位置
    move_group.setStartStateToCurrentState();
    std::string end_effector_link = move_group.getEndEffectorLink();
    RCLCPP_WARN(node->get_logger(), "End Effector Link: %s", end_effector_link.c_str());
    RCLCPP_INFO(node->get_logger(), "moved to init pose,start draw...");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose("sgr532/link_grasping_frame").pose;
    double offset_x = current_pose.position.x;
    double offset_y = current_pose.position.y;
    double offset_z = current_pose.position.z;
    RCLCPP_ERROR(node->get_logger(), "Current pose: x=%.2f, y=%.2f, z=%.2f", offset_x, offset_y, offset_z); // 输出当前的位置

    // 5. 生成五角星外角点位置
    std::vector<std::vector<double>> points = generate_star_outer_path(offset_x, offset_y, offset_z, 0.1);
    for (const auto& point : points) {
        RCLCPP_ERROR(node->get_logger(), "Point: x=%.2f, y=%.2f, z=%.2f", point[0], point[1], point[2]); // 输出生成的路径
    }

    // 6. 赋值五角星运动路径
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (const auto& point : points)
    {
        geometry_msgs::msg::Pose end_point;
        end_point.position.x = point[0];
        end_point.position.y = point[1];
        end_point.position.z = point[2];
        end_point.orientation = current_pose.orientation; // 保持当前的方向

        waypoints.push_back(end_point);
    }

    // 7. 执行路径规划并执行
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01; // 末端执行器步长
    const double jump_threshold = 0.0;

    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);
    if (fraction > 0.0)
    {
        RCLCPP_INFO(node->get_logger(), "Cartesian path computed successfully. Moving the arm.");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory = trajectory;
        // 尝试执行计划
        auto execute_result = move_group.execute(plan);
        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "Execution succeeded");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Execution failed with code: %d", execute_result.val);
        }
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Failed to compute a Cartesian path.");
    }
}


int main(int argc, char **argv)
{
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("draw_gcode",node_options);

    // Get parameters
    std::string gcode_file = node->get_parameter("gcode_file").as_string();
    std::vector<double> init_pose = node->get_parameter("init_pose").as_double_array();
    double pen_offset_up = node->get_parameter("pen_offset_up").as_double();
    double pen_offset_down = node->get_parameter("pen_offset_down").as_double();
    double x_offset = node->get_parameter("x_offset").as_double();
    double y_offset = node->get_parameter("y_offset").as_double();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    draw_gcode(node, gcode_file, init_pose, pen_offset_up, pen_offset_down, x_offset, y_offset);

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
