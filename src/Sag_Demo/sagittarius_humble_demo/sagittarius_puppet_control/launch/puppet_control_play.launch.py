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

    robot_model = LaunchConfiguration("robot_model")
    sagittarius_description_dir = get_package_share_directory('sagittarius_descriptions')
    sagittarius_arm_sdk_dir = get_package_share_directory('sdk_sagittarius_arm')
    sagittarius_puppet_control = get_package_share_directory('sagittarius_puppet_control')
    bag_name = LaunchConfiguration('bag_name')
    # 获取当前时间并格式化为字符串
    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_path = sagittarius_puppet_control + '/bag/' + robot_model.perform(context) + '_bag_' + bag_name.perform(context)

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')

    
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('sagittarius_descriptions'), 'urdf', "sgr532.urdf.xacro"]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    description_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sagittarius_description_dir, 'launch',
                                                       'sagittarius_description.launch.py')),
            launch_arguments={'robot_model': robot_model_launch_arg,
                            #   'use_joint_pub_gui': False,
                             }.items()),
    ])

    sdk_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sagittarius_arm_sdk_dir, 'launch',
                                                       'sagittarius_arm_sdk.launch.py')),
            launch_arguments={  'robot_model': robot_model_launch_arg,
                                'robot_name': robot_name_launch_arg,
                                # 'just_rviz_control': 'true',
                             }.items()),
    ])

    sag_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sagittarius_arm_sdk_dir, 'launch', 'sagittarius_description_true.launch.py')
        ),
        launch_arguments={
            'robot_model': robot_model,
            'robot_name': robot_name_launch_arg,
            'use_joint_pub_gui': 'false',
        }.items(),
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

    puppet_play = TimerAction(
        period=2.0,  # 延迟时间（秒）
        actions=[
            Node(
                package='sagittarius_puppet_control',
                executable='puppet_control_play.py',
                name='puppet_control_play',
                output='screen',
                namespace='/sgr532',
                # parameters=[
                #     {'robot_model': robot_model_launch_arg,
                #      'robot_name': robot_name_launch_arg,
                #      'use_sim_time': False,}
                # ],
            )
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=robot_model_launch_arg,
        parameters=[{
            'use_sim_time': False,
        }],
        output={'both': 'log'},
    )

    # 延迟 5 秒后执行 ros2 bag play
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
            # description_group,
            # sdk_group,
            # robot_state_publisher_node,
            # joint_state_publisher_node,
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
            default_value=LaunchConfiguration('robot_model'),
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
