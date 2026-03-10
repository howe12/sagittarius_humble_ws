# Copyright (c) 2024 NXROBO
#
# /* Author: haijie.huo */
# /* email: haijie.huo@nxrobo.com */
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):

    rviz_config_file = PathJoinSubstitution([FindPackageShare("robot_farming"), 'rviz', 'leo_farming.rviz'])
    robot_model = LaunchConfiguration("robot_model")
    app_dir = os.path.join(get_package_share_directory("rs_cam_calibration"), "scripts")
    controllers_config = load_yaml('sagittarius_moveit','config/controllers/sgr532_controllers.yaml')

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=robot_model.perform(context), package_name="sagittarius_moveit"
        )
        .robot_description_semantic(file_path=get_package_share_directory("sagittarius_moveit")
            + "/config/sgr532.srdf.xacro")
        .robot_description(file_path=get_package_share_directory("sagittarius_moveit")
            + "/config/sgr532.urdf.xacro"
            )
        .planning_pipelines("ompl", ["ompl",  "pilz_industrial_motion_planner"])
        .moveit_cpp(
            file_path=get_package_share_directory("sagittarius_moveit")
            + "/config/moveit_cpp.yaml"
        )
        .planning_scene_monitor(publish_robot_description=True,publish_robot_description_semantic=True)
        .to_moveit_configs()
        )
    
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('sagittarius_descriptions'), 'urdf', "sgr532.urdf.xacro"]
            ),
            # ' ',
            # 'namespace:=',
            # robot_name_launch_arg,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Launch the main launcher with servo
    sagittarius_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("sagittarius_moveit"),'/launch','/demo_true.launch.py']),
        launch_arguments={
            'robot_model': robot_model,
            'device_type':'actual'
        }.items(),
    )

    moveit_controllers = {
        'moveit_simple_controller_manager':
            controllers_config,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # 机械臂运动规划程序
    motion_planning = Node(
        package='sagittarius_py_demo',
        executable='motion_planning_api_demo', 
        # namespace= '/sgr532',
        parameters=[
            moveit_config.to_dict(),
            {'planning_scene_monitor_options': 
                {
                    'robot_description':'robot_description',
                    'joint_state_topic':'/sgr532/joint_states',
                },
                "use_sim_time": False,    
            },
            moveit_controllers,
            ], 
        # remappings = remappings,
        )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': False,}
        ],
        namespace='/sgr532',
        output={'both': 'screen'},
    )

    start_app = ExecuteProcess(
        cmd=["cd {} && python3 camera_calibration.py".format(app_dir)],
        shell=True,
        output="screen",
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    # 单相机
    camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("camera_driver_transfer"), '/launch','/d435.launch.py']),
            launch_arguments={'dp_rgist': "true",}.items(),)

    nodes_to_start =[
            sagittarius_moveit_launch,   
            # motion_planning,
            start_app,
            robot_state_publisher_node,
            # camera_launch,
            # rviz_node,
        ]
    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_model",
            default_value="sgr532",
            description='选择加入射手座的型号,例如 robot_model:=sgr532',
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
