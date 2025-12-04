from setuptools import setup
import os
from glob import glob

package_name = 'lunabot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aids',
    maintainer_email='your_email@email.com',
    description='Navigation package for LunaBot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'isaac_sim_bridge = lunabot_navigation.isaac_sim_bridge:main',
            'pointcloud_to_scan = lunabot_navigation.pointcloud_to_scan:main',
            'autonomous_patrol = lunabot_navigation.autonomous_patrol:main',
            'frontier_explorer = lunabot_navigation.frontier_explorer:main',
        ],
    },
)

