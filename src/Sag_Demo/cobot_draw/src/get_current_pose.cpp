#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
//https://answers.ros.org/question/417209/how-to-extract-position-of-the-gripper-in-ros2moveit2/
int main(int argc, char **argv)
{
    // Initialize the ROS node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("get_current_pose_node");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner = std::thread([&executor]() {executor.spin();});

    // Set up MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface move_group(node, "aubo_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Allow some time for RViz to come up
    // rclcpp::sleep_for(std::chrono::seconds(5));

    // Get the current pose of the end-effector
    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose("wrist3_Link");
    std::vector<double> joint_values = move_group.getCurrentJointValues();
    // Set up the MoveGroupInterface
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);
    // Print the current pose
    
    RCLCPP_INFO(node->get_logger(), "Current Joint Values:");
    for (size_t i = 0; i < joint_values.size(); ++i)
    {
        RCLCPP_INFO(node->get_logger(), "Joint %zu: %f", i + 1, joint_values[i]);
    }
    RCLCPP_INFO(node->get_logger(), "Current Pose:");
    RCLCPP_INFO(node->get_logger(), "Position - x: %f, y: %f, z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    RCLCPP_INFO(node->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f", current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
    
    // std::vector<double> init_pose = {-1.437709, -0.988911, 1.323454, 0.748714, 1.567814, -0.012193}; 
    // move_group.setJointValueTarget(init_pose);
    // moveit::planning_interface::MoveGroupInterface::Plan init_plan;
    // bool success = (move_group.plan(init_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if (success)
    // {
    //     move_group.execute(init_plan);
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to move to initial pose");
    // }
    //  RCLCPP_INFO(node->get_logger(), "move end");
    // Shutdown the ROS node
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
