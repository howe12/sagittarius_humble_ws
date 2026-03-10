import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('cobot_draw'),
        'config',
        'config.yaml')

    with open(config_file, 'r') as file:
        config_params = yaml.safe_load(file)
    gcode_file_path = os.path.join(get_package_share_directory('cobot_draw'), 'config', config_params['gcode_file'])
    config_params['gcode_file'] = gcode_file_path
    return LaunchDescription([
        Node(
            package='cobot_draw',
            executable='draw_gcode',
            name='draw_gcode',
            output='screen',
            parameters=[config_params]
        )
    ])
