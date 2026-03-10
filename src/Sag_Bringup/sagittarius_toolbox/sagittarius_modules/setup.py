from setuptools import (
    find_packages,
    setup,
)

from setuptools import setup

package_name = 'sagittarius_modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude='test'),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spark',
    maintainer_email='litian.zhuang@nxrobo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
