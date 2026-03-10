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
import glob
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
from launch.actions import LogInfo

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def list_tty_acm():
    """列出所有 /dev/sagittarius_* 设备"""
    devices = glob.glob("/dev/sagittarius_*")
    if not devices:
        print("未找到 /dev/sagittarius_* 设备！")
        return None
    print("检测到以下 /dev/sagittarius_* 设备：")
    for i, dev in enumerate(devices):
        print(f"{i}: {dev}")
    return devices


def launch_setup(context, *args, **kwargs):

    # 1.参数声明
    robot_model_launch_arg = LaunchConfiguration('robot_model')
    sagittarius_arm_sdk_dir = get_package_share_directory('sdk_sagittarius_arm')
    sagittarius_puppet_control = get_package_share_directory('sagittarius_puppet_control')
    rviz_dir = os.path.join(sagittarius_puppet_control, "rviz", "puppet_control.rviz")
    

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

    # 2.获取端口号信息,并创建映射字典
    i = 0
    robot_name_map = {}
    devices = list_tty_acm()
    for i, dev in enumerate(devices):
        if i == 0:
            robot_name_map[f'{dev}'] = f'sgr532_master' 
        else:   
            robot_name_map[f'{dev}'] = f'sgr532_follow_{i}'    # 在robot_model_map中添加新的键值对
    print("robot_name_map:", robot_name_map) # 打印映射字典

    # 3.建立机械臂驱动组
    group_actions = []
    for idx, (dev, ns) in enumerate(robot_name_map.items()):
        just_rviz_control = 'false' if idx == 0 else 'true'  # 第一个为 false，其它为 true
        group = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sagittarius_arm_sdk_dir, 'launch', 'sagittarius_description_true.launch.py')
                ),
                launch_arguments={
                    'robot_model': LaunchConfiguration('robot_model'),
                    'robot_name': ns,
                    'namespace': ns,
                    'serial_port': dev,
                    'use_joint_pub_gui': 'false',
                    'use_rviz': 'false',
                    'just_rviz_control': just_rviz_control,
                }.items()
            )
        ])
        group_actions.append(group)

    # 4.主机械臂控制命令发布程序
    puppet_master_node = TimerAction(
        period=8.0,  # 延迟时间（秒）
        actions=[
            Node(
                package='sagittarius_puppet_control',
                executable='puppet_control_single_node',
                name='puppet_master_node',
                output='screen',
                namespace=list(robot_name_map.values())[0],
                parameters=[
                    {'robot_model': robot_model_launch_arg,
                     'robot_name': list(robot_name_map.values())[0],
                     'use_sim_time': False,}
                ],
            )
        ]
    )


    # 5.从机械臂的跟随程序
    puppet_play_groups = []
    for dev, ns in list(robot_name_map.items())[1:]:
        group = GroupAction([
            TimerAction(
                period=8.0,  # 延迟时间（秒）
                actions=[
                    Node(
                        package='sagittarius_puppet_control',
                        executable='puppet_slaver_play.py',
                        name=f'puppet_follow_play_{ns}',
                        output='screen',
                        namespace=ns,
                    )
                ]
            )
        ])
        puppet_play_groups.append(group)

    # 6.启动rviz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # namespace=list(robot_name_map.values())[0],
        arguments=[
            '-d', rviz_dir,
        ],
        output={'both': 'screen'},
    )

    # 7.TF坐标转换，/sgr532_master和/sgr532_follow_1之间的坐标转换
    robot_name_list = list(robot_name_map.values())
    tf_groups = []
    for idx in range(len(robot_name_list)):
        base_frame = 'world' if idx == 0 else robot_name_list[idx-1] + "/base_link" # 第一个为 false，其它为 true
        tf_group = GroupAction([
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher_' + str(idx),
                        arguments=[
                            '0.0', '0.3', '0.0', '0.0', '0.0', '0.0',
                            base_frame,
                            robot_name_list[idx] + "/base_link",
                        ]
                    )
                ]
            )
        ])
        tf_groups.append(tf_group)



    nodes_to_start = group_actions + [puppet_master_node,rviz2_node] + puppet_play_groups+tf_groups
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
