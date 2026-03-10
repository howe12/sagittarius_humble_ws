#
#   Copyright (c) 2023, NXROBO Ltd.
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#
#   Authors: Litian Zhuang <litian.zhuang@nxrobo.com>
#

from sagittarius_modules.sgr_common import (
    get_sagittarius_arm_models,
)
from sagittarius_modules.sgr_launch import (
    declare_sagittarius_arm_robot_description_launch_arguments,
)
from sagittarius_modules.sgr_launch.sgr_launch import determine_use_sim_time_param
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    use_joint_pub_launch_arg = LaunchConfiguration('use_joint_pub')
    use_joint_pub_gui_launch_arg = LaunchConfiguration('use_joint_pub_gui')
    rvizconfig_launch_arg = LaunchConfiguration('rvizconfig')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    device_type_launch_arg = LaunchConfiguration('device_type')
    robot_name = LaunchConfiguration('robot_name').perform(context)
    # sets use_sim_time parameter to 'true' if using gazebo hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        device_type_launch_arg=device_type_launch_arg
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_launch_arg,
            'use_sim_time': use_sim_time_param,
        }],
        namespace=robot_name_launch_arg,
        output={'both': 'screen'},
    )

    joint_state_publisher_node = Node(
        condition=IfCondition(use_joint_pub_launch_arg),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=robot_name_launch_arg,
        parameters=[{
            'use_sim_time': use_sim_time_param,
        }],
        output={'both': 'log'},
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(use_joint_pub_gui_launch_arg),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=robot_name_launch_arg,
        output={'both': 'log'},
    )

    rviz2_node = Node(
        condition=IfCondition(use_rviz_launch_arg),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=robot_name_launch_arg,
        arguments=[
            '-d', rvizconfig_launch_arg,
        ],
        parameters=[{
            'use_sim_time': use_sim_time_param,
        }],
        output={'both': 'log'},
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            default_value='sgr532',
            choices=get_sagittarius_arm_models(),
            description='射手座机械臂的模型类型，例如`sgr532`.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='sgr532',
            description=(
                '机器人的名称(默认与robot_model一致).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='如果是‘true’，启动RViz2.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joint_pub',
            default_value='false',
            choices=('true', 'false'),
            description='启动joint_state_publisher节点.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joint_pub_gui',
            default_value='false',
            choices=('true', 'false'),
            description='启动joint_state_publisher界面.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('sagittarius_descriptions'),
                'rviz',
                'sagittarius_description.rviz',
            ]),
            description='加载RVIZ2的配置文件.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description='告诉 ROS 节点请求时间以获取 Gazebo 发布的模拟时间，发布在 ROS 主题 /clock 上。',
        )
    )
    declared_arguments.extend(
        declare_sagittarius_arm_robot_description_launch_arguments(),
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
