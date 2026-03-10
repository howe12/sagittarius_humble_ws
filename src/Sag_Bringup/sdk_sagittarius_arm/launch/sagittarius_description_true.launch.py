import os
import yaml
from ament_index_python.packages import get_package_share_directory
from sagittarius_modules.sgr_common import (
    get_sagittarius_arm_models,
)
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  
        return None


def launch_setup(context, *args, **kwargs):
    sagittarius_description_dir = get_package_share_directory('sagittarius_descriptions')
    sagittarius_arm_sdk_dir = get_package_share_directory('sdk_sagittarius_arm')

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    joint_configs_launch_arg = LaunchConfiguration('joint_configs')
    just_rviz_control_launch_arg = LaunchConfiguration('just_rviz_control')
    use_joint_pub_gui_launch_arg = LaunchConfiguration('use_joint_pub_gui')
    use_rviz_arg = LaunchConfiguration('use_rviz')

    serial_port_launch_arg = LaunchConfiguration('serial_port')
    exit_free_torque_launch_arg = LaunchConfiguration('exit_free_torque')


    description_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sagittarius_description_dir, 'launch',
                                                       'sagittarius_description.launch.py')),
            launch_arguments={'robot_model': robot_model_launch_arg,
                              'robot_name': robot_name_launch_arg,
                              'use_rviz_launch_arg': use_rviz_arg,
                              'use_joint_pub_gui': use_joint_pub_gui_launch_arg,
                             }.items())

    sdk_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sagittarius_arm_sdk_dir, 'launch',
                                                       'sagittarius_arm_sdk.launch.py')),
            launch_arguments={  'robot_model': robot_model_launch_arg,
                                'robot_name': robot_name_launch_arg,
                                'joint_configs': joint_configs_launch_arg,
                                'just_rviz_control': just_rviz_control_launch_arg,
                                'serial_port': serial_port_launch_arg,
                                'exit_free_torque': exit_free_torque_launch_arg,
                             }.items()),
    ])


    return [
        description_group,
        sdk_group,
    ]

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            default_value='sgr532',
            choices=get_sagittarius_arm_models(),
            description='选择加入射手座的型号,例如 robot_model:=sgr532 ',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='sgr532',
            description=(
                '默认跟robot_model一样的，也可用其它的。.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'joint_configs',
            default_value=[PathJoinSubstitution([
                FindPackageShare('sdk_sagittarius_arm'),
                'cfg',
                LaunchConfiguration('robot_model')]),
                '.yaml'
            ],
            description="sgr机械臂的yaml路径",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'serial_port',
            default_value="/dev/sagittarius_0",
            description=(
                '默认为/dev/sagittarius，或者选择/dev/ttyACMx'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'just_rviz_control',
            default_value='true',
            choices=('true', 'false'),
            description=(
                '只使用rviz控制,此时机械臂本身不发布当前的关节状态。'
            )
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description=(
                '是否显示rviz，默认显示rviz可视化界面'
            )
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joint_pub_gui',
            default_value='true',
            choices=('true', 'false'),
            description='启动joint_state_publisher界面.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'exit_free_torque',
            default_value='false',
            choices=('true', 'false'),
            description=(
                '在退出程序时是否释放舵机，选择true时，注意选让舵机回归到sleep位置。'
            )
        )
    )



    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])