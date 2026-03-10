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
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument(
            'robot_model',default_value='sgr532',choices=get_sagittarius_arm_models(),
            description='选择加入射手座的型号,例如 robot_model:=sgr532 ',
        )
    )
    declared_arguments.append(DeclareLaunchArgument(
            'robot_name',default_value=LaunchConfiguration('robot_model'),
            description=('默认跟robot_model一样的，也可用其它的。.'),
        )
    )
    declared_arguments.append(DeclareLaunchArgument(
            'joint_configs', default_value=[PathJoinSubstitution([FindPackageShare('sdk_sagittarius_arm'),'cfg',
                LaunchConfiguration('robot_model')]), '.yaml'],
            description="sgr机械臂的yaml路径",
        )
    )
    declared_arguments.append(DeclareLaunchArgument(
            'serial_port',default_value="/dev/ttyACM0",
            description=(
                '默认为/dev/sagittarius，或者选择/dev/ttyACMx' ),
        )
    )
    declared_arguments.append(DeclareLaunchArgument(
            'just_rviz_control',default_value='true',choices=('true', 'false'),
            description=('只使用rviz控制,此时机械臂本身不发布当前的关节状态。' )
        )
    )
    declared_arguments.append(DeclareLaunchArgument(
            'use_joint_pub_gui',default_value='true',choices=('true', 'false'),
            description='启动joint_state_publisher界面.',
        )
    )
    declared_arguments.append(DeclareLaunchArgument(
            'exit_free_torque',default_value='false',choices=('true', 'false'),
            description=('在退出程序时是否释放舵机，选择true时，注意选让舵机回归到sleep位置。')
        )
    )
    declared_arguments.append(DeclareLaunchArgument(
            'controllers_file',default_value='sgr532_ros_controllers.yaml',
            description=('机器人控制文件')
        )
    )
    declared_arguments.append(DeclareLaunchArgument(
            'sgr532_controllers_file',default_value='sgr532_controllers.yaml',
            description=('机器人控制文件')
        )
    )
    declared_arguments.extend(
        declare_sagittarius_arm_robot_description_launch_arguments(
            device_type='actual',
        )
    )
    # declared_arguments.extend(
    #     declare_sagittarius_arm_robot_description_launch_arguments()
    # )

    sagittarius_description_dir = get_package_share_directory('sagittarius_descriptions')
    sagittarius_arm_sdk_dir = get_package_share_directory('sdk_sagittarius_arm')
    sagittarius_moveit_config_dir = get_package_share_directory('sagittarius_humble_moveit')
    controllers_config = load_yaml('sagittarius_humble_moveit','config/controllers/sgr532_controllers.yaml')

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    joint_configs_launch_arg = LaunchConfiguration('joint_configs')
    just_rviz_control_launch_arg = LaunchConfiguration('just_rviz_control')
    use_joint_pub_gui_launch_arg = LaunchConfiguration('use_joint_pub_gui')
    serial_port_launch_arg = LaunchConfiguration('serial_port')
    exit_free_torque_launch_arg = LaunchConfiguration('exit_free_torque')
    controllers_file = LaunchConfiguration('controllers_file')
    sgr532_controllers_file = LaunchConfiguration('sgr532_controllers_file')
    robot_description_launch_arg = LaunchConfiguration('robot_description')


    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('sagittarius_descriptions'), 'urdf', "sgr532.urdf.xacro"]
            ),
            # ' ',
            # 'namespace:=',
            # robot_name_launch_arg,
        ]
    )
    # robot_description = {'robot_description': robot_description_content}
    robot_description = {'robot_description': robot_description_launch_arg}

    # Get SRDF via xacro
    # robot_description_semantic_content = ParameterValue(
    #         PathJoinSubstitution(
    #     [FindPackageShare('sagittarius_moveit'), "config", "sgr532.srdf"]
    # ),
    # value_type=str)
    # Command(
    #     [
    #         # PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         # " ",
    #         PathJoinSubstitution(
    #             # [FindPackageShare('sagittarius_humble_moveit_config'), "config", "sgr532.srdf"]
    #             [FindPackageShare('sagittarius_humble_moveit_config'), "config", "srdf","sgr532.srdf"]

    #         ),
    #     ]
    # ),value_type=str)

    # robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    

    config_path = PathJoinSubstitution([FindPackageShare(
        'sagittarius_humble_moveit'),
        'config',
    ])
    robot_description_semantic1 = {
        'robot_description_semantic':
            construct_sagittarius_arm_semantic_robot_description_command(
                robot_model="sgr532",
                config_path=config_path  
            ),
    }


    ros2_control_controllers_config_parameter_file = PathJoinSubstitution(
        [
            FindPackageShare('sagittarius_humble_moveit'),
            'config',
            'controllers',
            controllers_file,
        ]
    )

    sgr532_controllers_yaml = PathJoinSubstitution(
        [
            FindPackageShare('sagittarius_humble_moveit'),
            'config',
            'controllers',
            sgr532_controllers_file,
        ]
    )

    sgr532_controllers_yaml1 = load_yaml(
        'sagittarius_humble_moveit',
        'config/controllers/sgr532_controllers.yaml'
    )


    # Get planning parameters
    robot_description_planning_joint_limits = PathJoinSubstitution([
            FindPackageShare('sagittarius_humble_moveit'), "config", "joint_limits.yaml",
        ])
    robot_description_planning_cartesian_limits = PathJoinSubstitution([
            FindPackageShare('sagittarius_humble_moveit'), "config", "pilz_cartesian_limits.yaml",
        ])
    robot_description_kinematics = PathJoinSubstitution([
            FindPackageShare('sagittarius_humble_moveit'), "config", "kinematics_actual.yaml"
        ])

    planning_pipelines_config = PathJoinSubstitution([
            FindPackageShare('sagittarius_humble_moveit'), "config", "planning_pipelines_config.yaml",
        ])
    # ompl_planning_config = PathJoinSubstitution([
    #         FindPackageShare('sagittarius_humble_moveit'), "config", "ompl_planning.yaml",
    #     ])
    
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
        'sagittarius_humble_moveit', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_pipeline_yaml_file)

    # controllers_config = PathJoinSubstitution(
    #     [FindPackageShare('sagittarius_humble_moveit'),"config", "moveit_controllers.yaml"
    #     ])
    moveit_controllers = {
        'moveit_simple_controller_manager':
            controllers_config,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    remappings = [
        (
            'sgr532/get_planning_scene',
            '/sgr532/get_planning_scene'
        ),
        (
            '/arm_controller/follow_joint_trajectory',
            '/sgr532/arm_controller/follow_joint_trajectory'
        ),
        (
            '/gripper_controller/follow_joint_trajectory',
            '/sgr532/gripper_controller/follow_joint_trajectory'
        ),
    ]


    description_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sagittarius_description_dir, 'launch',
                                                       'sagittarius_description.launch.py')),
            launch_arguments={'robot_model': robot_model_launch_arg,
                              'use_joint_pub_gui': use_joint_pub_gui_launch_arg,
                             }.items()),
    ])

    sdk_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sagittarius_arm_sdk_dir, 'launch',
                                                       'sagittarius_arm_sdk.launch.py')),
            launch_arguments={  'robot_model': robot_model_launch_arg,
                                'robot_name': robot_name_launch_arg,
                             }.items()),
    ])

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        remappings=remappings,
        parameters=[
            robot_description,
            robot_description_semantic1,
            robot_description_kinematics,
            # robot_description_planning_cartesian_limits,
            ParameterFile(robot_description_planning_joint_limits, allow_substs=True),
            # planning_pipelines_config,
            ompl_planning_pipeline_config,
            # ompl_planning_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {
                'planning_scene_monitor_options': {
                    'robot_description':'robot_description',
                    'joint_state_topic':'/sgr532/joint_states',
                },
                "use_sim_time": False,
            },
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("sdk_sagittarius_arm"), 'rviz', 'moveit.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        remappings=remappings,
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic1,
            # robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            robot_description_kinematics,
            # planning_pipelines_config,
            ompl_planning_pipeline_config,
            # ompl_planning_config,
            {"use_sim_time": False},
        ],
        # condition=IfCondition(launch_rviz),
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=robot_name_launch_arg,
        # remappings=remappings,
        parameters=[
            # robot_description,
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
        # remappings=remappings,
        arguments=[
            '-c',
            '/sgr532/controller_manager',
            'arm_controller',
        ],
        output={'both': 'screen'},
    )

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        # remappings=remappings,
        arguments=[
            '-c',
            '/sgr532/controller_manager',
            'gripper_controller',
        ],
        output={'both': 'screen'},
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': False,}
        ],
        namespace=robot_name_launch_arg,
        output={'both': 'screen'},
    )



    nodes = [
        # description_group,
        sdk_group,
        move_group_node,
        rviz_node,

        controller_manager_node,
        spawn_arm_controller_node,
        spawn_gripper_controller_node,
        robot_state_publisher_node,
    ]

    return LaunchDescription(declared_arguments + nodes)