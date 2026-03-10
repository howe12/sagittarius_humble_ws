from setuptools import setup
import os
from glob import glob

setup(
    name='sagittarius_robot_control',
    version='0.1.0',
    packages=['sagittarius_robot_control'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/sagittarius_robot_control']),
        ('share/sagittarius_robot_control', ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='howe',
    maintainer_email='howe12@126.com',
    description='Sagittarius robot gesture control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gesture_player = sagittarius_robot_control.gesture_player:main',
        ],
    },
)
