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
    # controllers_config = load_yaml('sagittarius_humble_moveit','config/controllers/sgr532_controllers.yaml')

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name=robot_model.perform(context), package_name="sagittarius_humble_moveit"
        )
        .robot_description_semantic(file_path=get_package_share_directory("sagittarius_humble_moveit")
            + "/config/sgr532.srdf")
        .robot_description(file_path=get_package_share_directory("sagittarius_humble_moveit")
            + "/config/sgr532.urdf.xacro"
            )
        .planning_pipelines("ompl", ["ompl",  "pilz_industrial_motion_planner"])
        .planning_scene_monitor(publish_robot_description=True,publish_robot_description_semantic=True)
        .to_moveit_configs()
        )
    
    # Launch the main launcher with servo
    sagittarius_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("sagittarius_humble_moveit"),'/launch','/demo.launch.py']),
    )

    moveit_controllers = {
        # 'moveit_simple_controller_manager':
        #     controllers_config,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
  
    config_file = os.path.join(get_package_share_directory('cobot_draw'),'config','config.yaml')

    with open(config_file, 'r') as file:
        config_params = yaml.safe_load(file)
    gcode_file_path = os.path.join(get_package_share_directory('cobot_draw'), 'config', config_params['gcode_file'])
    config_params['gcode_file'] = gcode_file_path
    
    # 绘制程序
    draw_gcode = Node(
        package='cobot_draw',
        executable='draw_gcode',
        name='draw_gcode',
        output='screen',
        parameters=[config_params,
            moveit_config.to_dict(),
            {'planning_scene_monitor_options': 
                {
                    'robot_description':'robot_description',
                    'joint_state_topic':'/sgr532/joint_states',
                },
                "use_sim_time": False,    
            },
            moveit_controllers,
            ]
    )
    
    # 延时启动 hsv_detect 节点（延时 5 秒）
    delayed_draw_gcode = TimerAction(
        period=5.0,  # 延时 5 秒启动
        actions=[draw_gcode],
    )

    nodes_to_start =[
            sagittarius_moveit_launch,   
            delayed_draw_gcode,
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
