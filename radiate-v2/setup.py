from glob import glob
import os
from setuptools import setup

package_name = 'radiate_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # ament resource index marker
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ROS 2 player for the RADIATE adverse-weather dataset',
    license='MIT',
    entry_points={
        'console_scripts': [
            'player = radiate_ros2.radiate_player_node:main',
        ],
    },
)
