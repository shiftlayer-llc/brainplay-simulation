from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'codenames_game'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'geometry_msgs',
        'sensor_msgs',
        'requests',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='4-Player Codenames game implementation for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orchestrator_node = codenames_game.orchestrator_node:main',
            'player_node = codenames_game.player_node:main',
            'isaac_bridge_node = codenames_game.isaac_bridge_node:main',
            # Legacy nodes (kept for compatibility)
            'spymaster_node = codenames_game.spymaster_node:main',
            'finder_node = codenames_game.finder_node:main',
        ],
    },
)