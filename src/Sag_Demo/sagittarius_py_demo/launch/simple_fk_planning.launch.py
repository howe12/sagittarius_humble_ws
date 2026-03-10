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

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):

    # 获取机器人模型及控制器配置
    robot_model = LaunchConfiguration("robot_model")
    controllers_config = load_yaml('sagittarius_moveit',f'config/controllers/{robot_model.perform(context)}_controllers.yaml')

    # 获取机械臂 MoveIt 配置信息
    moveit_config = (
        # 创建 MoveIt 配置构建器
        MoveItConfigsBuilder(
            # 设置机器人名称（动态获取，基于 robot_model.perform(context)）
            robot_name=robot_model.perform(context),
            # 指定 MoveIt 配置包的名称
            package_name="sagittarius_moveit"
        )
        # 加载机器人描述文件的 SRDF（语义机器人描述格式），用于定义机械臂的运动规划约束
        .robot_description_semantic(
            # 拼接 SRDF 文件路径
            file_path=get_package_share_directory("sagittarius_moveit")
            + f"/config/{robot_model.perform(context)}.srdf.xacro"
        )
        # 加载机器人描述文件的 URDF（统一机器人描述格式），用于定义机械臂的结构和关节信息
        .robot_description(
            # 拼接 URDF 文件路径
            file_path=get_package_share_directory("sagittarius_moveit")
            + f"/config/{robot_model.perform(context)}.urdf.xacro"
        )
        # 配置规划管道，使用 OMPL（开放运动规划库）和 Pilz 工业运动规划器
        .planning_pipelines(
            "ompl",  # 默认规划管道名称
            ["ompl", "pilz_industrial_motion_planner"]  # 支持的规划器列表
        )
        # 加载 MoveIt CPP 配置，用于 MoveIt 的 C++ API 配置参数
        .moveit_cpp(
            # 拼接 MoveIt CPP 配置文件路径
            file_path=get_package_share_directory("sagittarius_moveit")
            + "/config/moveit_cpp.yaml"
        )
        # 配置规划场景监视器，设置是否发布机器人描述信息
        .planning_scene_monitor(
            publish_robot_description=True,  # 发布机器人描述
            publish_robot_description_semantic=True  # 发布语义机器人描述
        )
        # 转换为 MoveIt 配置对象
        .to_moveit_configs()
    )

    
    # 启动机械臂驱动
    sagittarius_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("sagittarius_moveit"),'/launch','/demo_true.launch.py']),
        launch_arguments={
            'robot_model': robot_model,
            'device_type':'actual'
        }.items(),
    )

    # 机械臂控制器配置
    moveit_controllers = {
        'moveit_simple_controller_manager':
            controllers_config,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # 机械臂正运动学规划程序
    motion_planning = Node(
        package='sagittarius_py_demo',
        executable='simple_fk_planning', 
        parameters=[
            moveit_config.to_dict(),
            {'planning_scene_monitor_options': 
                {
                    'robot_description':'robot_description',
                    'joint_state_topic':f"/{robot_model.perform(context)}/joint_states",
                },
                "use_sim_time": False,    
            },
            moveit_controllers,
            ], 
        )
    
    # 定义要启动的节点列表
    nodes_to_start =[
            sagittarius_moveit_launch,   
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
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
