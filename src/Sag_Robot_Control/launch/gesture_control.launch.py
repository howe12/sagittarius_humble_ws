#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sagittarius_robot_control',
            executable='gesture_player.py',
            name='gesture_player',
            output='screen',
            emulate_tty=True,
        ),
    ])
