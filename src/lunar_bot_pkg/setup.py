from setuptools import setup, find_packages

package_name = 'lunar_bot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aids',
    maintainer_email='aids@todo.todo',
    description='Autonomous lunar patrol with Nav2 and O2 monitoring',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lunar_patrol = lunar_bot_pkg.lunar_patrol:main',
        ],
    },
)

