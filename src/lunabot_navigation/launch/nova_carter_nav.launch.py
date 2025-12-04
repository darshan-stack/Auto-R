import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg = FindPackageShare("lunabot_navigation")
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    map_file = LaunchConfiguration('map', 
                                   default=PathJoinSubstitution([pkg, "maps", "luna_map.yaml"]))
    params_file = LaunchConfiguration('params', 
                                     default=PathJoinSubstitution([pkg, "config", "nav2_params.yaml"]))
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg, "maps", "luna_map.yaml"]),
        description='Full path to map yaml file'
    )
    
    declare_params = DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution([pkg, "config", "nav2_params.yaml"]),
        description='Full path to nav2 params yaml file'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2 for visualization'
    )
    
    # Static transform: map -> odom
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )
    
    # RViz2 for visualization
    rviz_config = PathJoinSubstitution([
        pkg,
        'rviz',
        'navigation.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    # Nav2 bringup with topic remapping
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_file,
            'params_file': params_file,
            'autostart': 'true',
            'remappings': "[('/odom', '/chassis/odom')]"
        }.items()
    )
    
    # Delayed start for navigation (give Isaac Sim time to start)
    delayed_nav = TimerAction(
        period=3.0,
        actions=[nav2_bringup]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params,
        declare_rviz,
        
        # Start components
        map_to_odom_tf,
        rviz_node,
        delayed_nav,
    ])
