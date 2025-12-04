from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    pkg = FindPackageShare("lunabot_navigation")

    map_file = PathJoinSubstitution([pkg, "maps", "luna_map.yaml"])
    params_file = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])

    # Static transform from map to odom (identity transform for initial localization)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "bringup_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "map": map_file,
            "params_file": params_file,
            "autostart": "true"
        }.items()
    )

    return LaunchDescription([
        map_to_odom_tf,
        bringup
    ])

