"""
LunaBot Full System Launch
Integrates Isaac Sim robot with ROS 2 Navigation Stack

Usage:
  ros2 launch lunabot_navigation lunabot_full.launch.py

Prerequisites:
  1. Isaac Sim running with ROS 2 bridge enabled
  2. Robot publishing /tf, /scan, /odom topics
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import launch

def generate_launch_description():
    
    pkg = FindPackageShare("lunabot_navigation")
    
    # Launch arguments
    use_slam = LaunchConfiguration('use_slam', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    map_file = LaunchConfiguration('map', 
                                   default=PathJoinSubstitution([pkg, "maps", "luna_map.yaml"]))
    params_file = LaunchConfiguration('params', 
                                     default=PathJoinSubstitution([pkg, "config", "nav2_params.yaml"]))
    
    # Declare launch arguments
    declare_use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Whether to use SLAM (true) or pre-built map (false)'
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
    
    # Navigation stack (delayed start to allow Isaac Sim topics to initialize)
    nav_bringup = TimerAction(
        period=3.0,  # Wait 3 seconds for Isaac Sim
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg, "launch", "nav_single.launch.py"])
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'map': map_file,
                    'params_file': params_file,
                    'autostart': 'true'
                }.items()
            )
        ]
    )
    
    # SLAM (optional - only if use_slam=true)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "slam.launch.py"])
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(use_slam)
    )
    
    return LaunchDescription([
        declare_use_slam,
        declare_map,
        declare_params,
        declare_rviz,
        
        # Start navigation and visualization
        rviz_node,
        nav_bringup,
        slam_launch
    ])
