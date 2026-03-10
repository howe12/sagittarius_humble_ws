
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sagittarius_cam_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share', package_name,'config'), glob(os.path.join('config','*yaml'))),
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
            'camera_calibration = sagittarius_cam_calibration.camera_calibration:main',
            'charuco_detect = sagittarius_cam_calibration.charuco_detect:main',
            'charuco_detect_tf = sagittarius_cam_calibration.charuco_detect_tf:main',
            'cam2apriltag = sagittarius_cam_calibration.cam2apriltag:main'
        ],
    },
)
