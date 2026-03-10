import os
import yaml
from ament_index_python.packages import get_package_share_directory
from sagittarius_modules.sgr_common import (
    get_sagittarius_arm_models,
)
from sagittarius_modules.sgr_launch import (
    construct_sagittarius_arm_semantic_robot_description_command,
    declare_sagittarius_arm_robot_description_launch_arguments,
    determine_use_sim_time_param,
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
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    sagittarius_arm_sdk_dir = get_package_share_directory('sdk_sagittarius_arm')

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    use_moveit_rviz_launch_arg = LaunchConfiguration('use_moveit_rviz')
    rviz_frame_launch_arg = LaunchConfiguration('rviz_frame')
    rviz_config_file_launch_arg = LaunchConfiguration('rviz_config_file')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    device_type_launch_arg = LaunchConfiguration('device_type')

    # 加载控制器配置文件
    ros2_control_controllers_config_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('sagittarius_moveit'),
            'config',
            'controllers',
            f'{robot_model_launch_arg.perform(context)}_ros_controllers.yaml',
        ]),
        allow_substs=True
    )

    # sets use_sim_time parameter to 'true' if using gazebo hardware
    use_sim_time_param = determine_use_sim_time_param(
        context=context,
        device_type_launch_arg=device_type_launch_arg
    )

    robot_description = {'robot_description': robot_description_launch_arg}

    config_path = PathJoinSubstitution([
        FindPackageShare('sagittarius_moveit'),
        'config',
    ])
    print(config_path)
    robot_description_semantic = {
        'robot_description_semantic':
            construct_sagittarius_arm_semantic_robot_description_command(
                robot_model=robot_model_launch_arg.perform(context),
                config_path=config_path #"/home/spark/sagittarius_humble_ws/install/sagittarius_moveit/share/sagittarius_moveit/config" 
            ),
    }

    # 拼接文件路径
    kinematics_file_path = PathJoinSubstitution([
        FindPackageShare('sagittarius_moveit'),
        'config',
        'kinematics_actual.yaml'
    ])

    # 将其传递给 Node 节点的参数配置
    kinematics_config_parameter_file = ParameterFile(
        param_file=kinematics_file_path,  # 传递拼接的路径
        allow_substs=True
    )

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin':
                'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error':
                0.1,
        }
    }

    ompl_planning_pipeline_yaml_file = load_yaml(
        'sagittarius_moveit', 'config/ompl_planning_true.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_pipeline_yaml_file)

    controllers_config = load_yaml(
        'sagittarius_moveit',
        f'config/controllers/{robot_model_launch_arg.perform(context)}_controllers.yaml'
    )

    config_joint_limits = load_yaml(
        'sagittarius_moveit',
        f'config/joint_limits/{robot_model_launch_arg.perform(context)}_joint_limits.yaml'
    )

    joint_limits = {
        'robot_description_planning': config_joint_limits,
    }

    moveit_controllers = {
        'moveit_simple_controller_manager':
            controllers_config,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution_parameters = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    sensor_parameters = {
        'sensors': [''],
    }

    remappings = [
        (
            f'{robot_name_launch_arg.perform(context)}/get_planning_scene',
            f'/{robot_name_launch_arg.perform(context)}/get_planning_scene'
        ),
        (
            '/arm_controller/follow_joint_trajectory',
            f'/{robot_name_launch_arg.perform(context)}/arm_controller/follow_joint_trajectory'
        ),
        (
            '/gripper_controller/follow_joint_trajectory',
            f'/{robot_name_launch_arg.perform(context)}/gripper_controller/follow_joint_trajectory'
        ),
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            kinematics_config_parameter_file,
            {
                'planning_scene_monitor_options': {
                    'robot_description':
                        'robot_description',
                    'joint_state_topic':
                        f'/{robot_name_launch_arg.perform(context)}/joint_states',
                },
                'use_sim_time': use_sim_time_param,
            },
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            trajectory_execution_parameters,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits,
            sensor_parameters,
        ],
        remappings=remappings,
        output={'both': 'screen'},
    )
    
    moveit_rviz_node = Node(
        condition=IfCondition(use_moveit_rviz_launch_arg),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rviz_config_file_launch_arg,
            '-f', rviz_frame_launch_arg,
        ],
        parameters=[
            kinematics_config_parameter_file,
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            {
             'use_sim_time': use_sim_time_param,
            },
        ],
        remappings=remappings,
        output={'both': 'log'},
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=robot_name_launch_arg,
        parameters=[
            {'robot_description': robot_description_launch_arg},
            ros2_control_controllers_config_parameter_file,
        ],
        output={'both': 'screen'},
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            f'/{robot_name_launch_arg.perform(context)}/controller_manager',
            'arm_controller',
        ],
        output={'both': 'screen'},
    )

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            f'/{robot_name_launch_arg.perform(context)}/controller_manager',
            'gripper_controller',
        ],
        output={'both': 'screen'},
    )

    sdk_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sagittarius_arm_sdk_dir, 'launch',
                                                       'sagittarius_arm_sdk.launch.py')),
            launch_arguments={  'robot_model': robot_model_launch_arg,
                                'robot_name': robot_name_launch_arg,
                             }.items()),
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_launch_arg,
            'use_sim_time': use_sim_time_param,
        }],
        namespace=robot_name_launch_arg,
        output={'both': 'log'},
    )
    return [

        sdk_group,
        move_group_node,
        moveit_rviz_node,

        controller_manager_node,
        spawn_arm_controller_node,
        spawn_gripper_controller_node,
        robot_state_publisher_node,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=get_sagittarius_arm_models(),
            description='选择加入射手座的型号,例如 robot_model:=sgr532 ',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                '默认跟robot_model一样的，也可用其它的。.'
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('sagittarius_moveit'),
                'config',
                'modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_moveit_rviz',
            default_value='true',
            choices=('true', 'false'),
            description="launches RViz with MoveIt's RViz configuration/",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_frame',
            default_value='world',
            description=(
                'defines the fixed frame parameter in RViz. Note that if `use_world_frame` is '
                '`false`, this parameter should be changed to a frame that exists.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('sagittarius_moveit'),
                'rviz',
                'sgr_moveit.rviz'
            ]),
            description='file path to the config file RViz should load.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock; this value is automatically set to `true` if'
                ' using Gazebo hardware.'
            )
        )
    )
    declared_arguments.extend(
        declare_sagittarius_arm_robot_description_launch_arguments(
            device_type='actual',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'load_configs',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'load_configs'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )

    declared_arguments.extend(
        declare_sagittarius_arm_robot_description_launch_arguments()
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
