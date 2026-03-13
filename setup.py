from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aurora_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amit',
    maintainer_email='bamit127@gmail.com',
    description='ROS2 driver package for Aurora vehicle CAN-bus control and monitoring',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_controller = aurora_driver.vehicle_controller:main',
            'mock_vehicle = aurora_driver.mock_aurora_vehicle_node:main',
        ],
    },
)