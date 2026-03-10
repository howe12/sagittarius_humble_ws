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
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription,TimerAction)
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from datetime import datetime
import shutil

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    bag_name = LaunchConfiguration('bag_name')

    sagittarius_arm_sdk_dir = get_package_share_directory('sdk_sagittarius_arm')
    sagittarius_puppet_control = get_package_share_directory('sagittarius_puppet_control')
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S") # 获取当前时间并格式化为字符串
    bag_path = sagittarius_puppet_control + '/bag/' + robot_model_launch_arg.perform(context) + '_bag_' + bag_name.perform(context)

    sag_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sagittarius_arm_sdk_dir, 'launch', 'sagittarius_description_true.launch.py')
        ),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'use_joint_pub_gui': 'false',
            'namespace': robot_name_launch_arg,
            'serial_port': '/dev/sagittarius_0',
        }.items(),
    )

    puppet_play = TimerAction(
        period=2.0,  # 延迟时间（秒）
        actions=[
            Node(
                package='sagittarius_puppet_control',
                executable='puppet_slaver_play.py',
                name='puppet_slaver_play',
                output='screen',
                namespace=robot_name_launch_arg,
            )
        ]
    )

    rosbag_play_process = TimerAction(
        period=8.0,  # 延迟 5 秒
        actions=[
            ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_path],
                output="screen"
            )
        ]
    )

    nodes_to_start =[
            sag_bringup,
            puppet_play,
            rosbag_play_process,          
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
            'robot_name',
            default_value="sgr532",
            description=(
                '默认跟robot_model一样的,也可用其它的。.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'bag_name',
            default_value="test",
            description=(
                '记录的bag名称。'
            ),
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
