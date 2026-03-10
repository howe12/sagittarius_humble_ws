"""
A launch file for running the motion planning python api tutorial
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch_param_builder import load_yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def launch_setup(context, *args, **kwargs):
    aubo_type = LaunchConfiguration("aubo_type")

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=aubo_type.perform(context), package_name="aubo_description"
        )
        .robot_description_semantic(file_path=get_package_share_directory("aubo_description")
            + "/srdf/aubo.srdf.xacro")
        # .robot_description(file_path=get_package_share_directory("aubo_description")
        #     + "/urdf/aubo_ES3.urdf.xacro"
        #     ,mappings={
        #             "aubo_type": aubo_type.perform(context),
        #             }
        #     )
        .planning_pipelines("ompl", ["ompl",  "pilz_industrial_motion_planner"])
        .moveit_cpp(
            file_path=get_package_share_directory("motion_planning_python_tutorial")
            + "/config/motion_planning_python_api_tutorial.yaml"
        )
        .planning_scene_monitor(publish_robot_description=True,publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="motion_planning_python_tutorial",
        executable=LaunchConfiguration("example_file"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "aubo_base"],
    )

    nodes_to_start =[
            moveit_py_node,
            # static_tf,
    ]
    return nodes_to_start


def generate_launch_description():
    # declare parameter for using robot ip
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_type",
            default_value="aubo_ES3",
            description='Description with aubo robot type.',
        )
    )
    declared_arguments.append(
            DeclareLaunchArgument(
        "example_file",
        default_value="llm_moveit_planning.py",
        description="Python API tutorial file name",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
