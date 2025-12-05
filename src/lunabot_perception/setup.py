from setuptools import find_packages, setup

package_name = 'lunabot_perception'

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
    description='Perception and hazard detection for LunaBot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hazard_detector = lunabot_perception.hazard_detector:main',
            'enhanced_hazard_detector = lunabot_perception.enhanced_hazard_detector:main',
            'control_panel = lunabot_perception.control_panel:main',
        ],
    },
)
