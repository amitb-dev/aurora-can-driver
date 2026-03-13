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
        # Include all launch files from the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amit',
    maintainer_email='bamit127@gmail.com',
    description='Aurora Vehicle Driver for ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Maps the command to the main function in your script
            'vehicle_controller = aurora_driver.vehicle_controller:main'
        ],
    },
)