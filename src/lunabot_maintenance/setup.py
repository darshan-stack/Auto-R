from setuptools import find_packages, setup

package_name = 'lunabot_maintenance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aids',
    maintainer_email='aids@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'env_monitor = lunabot_maintenance.env_monitor:main',
            'autonomous_patrol = lunabot_maintenance.autonomous_patrol:main',
            'teleop_keyboard = lunabot_maintenance.teleop_keyboard:main',
            'scan_relay = lunabot_maintenance.scan_relay:main',
            'slam_odom_bridge = lunabot_maintenance.slam_odom_bridge:main',
        ],
    },
)
