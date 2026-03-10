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
from launch.actions import TimerAction
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):

    robot_model = LaunchConfiguration("robot_model")
    camera_type = LaunchConfiguration("camera_type")
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
  
    # USB相机
    usb_cam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("usb_cam"), '/launch','/camera.launch.py']),)
    
    # 根据 camera_type 选择不同的话题
    image_topic = PythonExpression([
        "'/camera1/image_raw' if '", LaunchConfiguration("camera_type"), "' == 'usb_cam' else "
        "'/camera/camera/color/image_raw' if '", LaunchConfiguration("camera_type"), "' == 'd435' else "
        "'/depth_camera/depth/image_raw' if '", LaunchConfiguration("camera_type"), "' == 'depth_camera' else "
        "'/camera1/image_raw'"
    ])

    # HSV检测程序
    hsv_detect = Node(
        package='sagittarius_py_demo',
        executable='hsv_detect',
        parameters=[
            # 设置相机话题名称
            {'image_topic': image_topic},
        ],
    )

    # 机械臂运动规划程序
    motion_planning = Node(
        package='sagittarius_py_demo',
        executable='hsv_catch', 
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
        )
    
    # 延时启动 hsv_detect 节点（延时 5 秒）
    delayed_hsv_detect = TimerAction(
        period=10.0,  # 延时 5 秒启动
        actions=[hsv_detect],
    )

    nodes_to_start =[
            sagittarius_moveit_launch,   
            usb_cam_launch,
            delayed_hsv_detect,
            motion_planning,
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_type",
            default_value="usb_cam",
            description='选择使用的相机类型,例如 camera_type:=usb_cam',
        ),
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
