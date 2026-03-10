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

void savePointsToCSV(const std::vector<std::vector<double>>& points, const std::string& output_file_path)
{
    std::ofstream csvfile(output_file_path);
    if (!csvfile.is_open())
    {
        throw std::runtime_error("Could not create CSV file.");
    }

    // Write header
    csvfile << "X,Y,Z\n";

    // Write data
    for (const auto& point : points)
    {
        csvfile << point[0] << "," << point[1] << "," << point[2] << "\n";
    }

    csvfile.close();
}

// GCode解析函数，增加起始点偏移和抬笔落笔高度
std::vector<std::vector<double>> parseGCode(const std::string &file_path, double offset_x, double offset_y,double offset_z, double pen_offset_up, double pen_offset_down)
{
    std::vector<std::vector<double>> points;
    std::ifstream file(file_path);
    std::string line;
    bool pen_down = false;  // To track the state of the pen

    if (!file.is_open())
    {
        throw std::runtime_error("Could not open GCode file.");
    }

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string token;
        std::vector<double> point(3, 0.0); // Initialize with 0.0

        while (iss >> token)
        {
            if (token[0] == 'X')
            {
                point[0] = std::stod(token.substr(1)) * -0.001 + offset_x;
            }
            else if (token[0] == 'Y')
            {
                point[1] = std::stod(token.substr(1)) * -0.001 + offset_y;
            }
            else if (token == "M3")
            {
                pen_down = true;
            }
            else if (token == "M5")
            {
                pen_down = false;
            }
        }

        // Apply Z-axis offset based on pen state
        point[2] = pen_down ? offset_z + pen_offset_down : offset_z + pen_offset_up;

        // Check if both X and Y are zero before adding the point
        if (point[0] != 0.0 || point[1] != 0.0) {
            points.push_back(point);
        }
    }

    return points;
}


void draw_gcode(const rclcpp::Node::SharedPtr& node, const std::string &gcode_file_path, const std::vector<double>& init_pose,
                double pen_offset_up, double pen_offset_down, double x_offset, double y_offset)
{

    // Set up MoveIt interfaces
    // moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
    moveit::planning_interface::MoveGroupInterface move_group(node, "sagittarius_arm");

    // Add a delay to ensure that the robot state is fully updated
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Set up the MoveGroupInterface
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

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
    RCLCPP_INFO(node->get_logger(), "moved to init pose,start draw...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose("sgr532/link_grasping_frame").pose;
    double offset_x = current_pose.position.x;
    double offset_y = current_pose.position.y;
    double offset_z = current_pose.position.z;
    // 解析GCode文件
    auto points = parseGCode(gcode_file_path, offset_x, offset_y,offset_z, pen_offset_up, pen_offset_down);

    savePointsToCSV(points,"output_points.csv");
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

        // // 获取当前 MoveGroup 的运动学配置
        // auto kinematics_config = move_group.getKinematicsSolver();
        // // 检查运动学配置是否有效
        // if (kinematics_config.empty()) {
        // RCLCPP_ERROR(node->get_logger(), "No kinematics solver instantiated for group '%s'", move_group.getName());
        // } else {
        // RCLCPP_INFO(node->get_logger(), "Kinematics solver for group '%s' is correctly instantiated.", move_group.getName());
        // }
        
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

    // Declare parameters
    // node->declare_parameter<std::string>("gcode_file", "/path/to/your/gcode/file.gcode");
    // node->declare_parameter<std::vector<double>>("init_pose", {0.0, -1.57, 1.57, 0.0, 1.57, 0.0});
    // node->declare_parameter<double>("pen_offset_up", 0.02);
    // node->declare_parameter<double>("pen_offset_down", 0.0);
    // node->declare_parameter<double>("x_offset", 0.0);
    // node->declare_parameter<double>("y_offset", 0.0);

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
