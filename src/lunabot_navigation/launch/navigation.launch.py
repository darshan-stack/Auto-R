from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    navigation_dir = get_package_share_directory('lunabot_navigation')

    bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    params_file = os.path.join(navigation_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments=[
                ('use_sim_time', 'True'),
                ('params_file', params_file),
                ('autostart', 'True'),
                ('slam', 'True'),
                ('map_subscribe_transient_local', 'True'),
                ('map', '/tmp/empty_map.yaml')  # <=== ADD THIS LINE (Dummy path to fix error)
            ]
        )
    ])
