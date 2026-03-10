from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sagittarius_py_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share', package_name,'rviz'), glob(os.path.join('rviz','*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='howe12@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_planning_api_demo = sagittarius_py_demo.motion_planning_api_demo:main',
            'simple_fk_planning = sagittarius_py_demo.simple_fk_planning:main',
            'simple_ik_planning = sagittarius_py_demo.simple_ik_planning:main',
            'hsv_catch = sagittarius_py_demo.hsv_catch:main',
            'hsv_detect = sagittarius_py_demo.hsv_detect:main',
            'ros_roi_hsv = sagittarius_py_demo.ros_roi_hsv:main',
            'eye_out_hsv_detect = sagittarius_py_demo.eye_out_hsv_detect:main',
            'eye_out_hsv_catch = sagittarius_py_demo.eye_out_hsv_catch:main',
            'yolo_catch = sagittarius_py_demo.yolo_catch:main',
            'sag_detect = sagittarius_py_demo.sag_detect:main',
            'sag_eye_out_motion = sagittarius_py_demo.sag_eye_out_motion:main',
            'meal_catch = sagittarius_py_demo.meal_catch:main'
        ],
    },
)
