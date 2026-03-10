# Copyright (c) 2025 NXROBO
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
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription,TimerAction)
from launch.actions import DeclareLaunchArgument, ExecuteProcess
import shutil
from launch.actions import LogInfo

def launch_setup(context, *args, **kwargs):

    # 1.配置参数
    robot_model = LaunchConfiguration("robot_model")
    sagittarius_arm_sdk_dir = get_package_share_directory('sdk_sagittarius_arm')
    sagittarius_puppet_control = get_package_share_directory('sagittarius_puppet_control')
    bag_name = LaunchConfiguration('bag_name')
    bag_path = sagittarius_puppet_control + '/bag/' + robot_model.perform(context) + '_bag_' + bag_name.perform(context)
    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')

     # 2.加载机器人描述文件
    robot_description_content = Command(
        [
            PathJoinSubstitution ([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('sagittarius_descriptions'), 'urdf', "sgr532.urdf.xacro"]
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}


    # 3.启动机械臂驱动sdk
    sdk_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sagittarius_arm_sdk_dir, 'launch',
                                                       'sagittarius_arm_sdk.launch.py')),
            launch_arguments={  'robot_model': robot_model_launch_arg,
                                'robot_name': robot_name_launch_arg,
                             }.items()
        )

    # 4.启动robot_state_publisher
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

    # 5.启动释放舵机程序
    ros2bag_record_node = TimerAction(
        period=8.0,  # 延迟时间（秒）
        actions=[
            Node(
                package='sagittarius_puppet_control',
                executable='ros2bag_record_control',
                name='ros2bag_record_control',
                output='screen',
                namespace=robot_name_launch_arg,
                parameters=[
                    {'robot_model': robot_model_launch_arg,
                     'robot_name': robot_name_launch_arg,
                     'use_sim_time': False,}
                ],
            )
        ]
    )

    
    if os.path.exists(bag_path): # 如果 bag 目录已存在，先删除
        shutil.rmtree(bag_path)  # 递归删除整个目录

    # 6. 启动 ros2 bag record
    rosbag_record_process = TimerAction(
        period=5.0,  # 延迟时间（秒）
        actions=[
            LogInfo(msg=["Bag path is: ", bag_path]),
            LogInfo(msg=["Joint command topic: /", robot_name_launch_arg.perform(context), "/joint_states"]),

            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "record",
                    "-o", [bag_path],
                    f"/{robot_name_launch_arg.perform(context)}/joint_states",
                ],
                output="screen"
            )
        ]
    )

    # 7. 启动节点
    nodes_to_start =[
            sdk_group,
            robot_state_publisher_node,
            ros2bag_record_node,
            rosbag_record_process,
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
