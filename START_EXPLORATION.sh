#!/bin/bash
# Full Autonomous Exploration System - Complete Command List
# Run after pressing Play in Isaac Sim

echo "🚀 Starting Full Autonomous Exploration System"
echo "================================================"
echo ""

# Terminal 1: Environmental Monitor
echo "Terminal 1 - Environmental Monitor:"
echo "cd /home/aids/lunabot_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run lunabot_maintenance env_monitor"
echo ""

# Terminal 2: Scan Relay
echo "Terminal 2 - Scan Relay:"
echo "cd /home/aids/lunabot_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run lunabot_maintenance scan_relay"
echo ""

# Terminal 3: SLAM Toolbox
echo "Terminal 3 - SLAM Toolbox:"
echo "cd /home/aids/lunabot_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/aids/lunabot_ws/src/lunabot_navigation/config/slam_params.yaml use_sim_time:=true"
echo ""

# Terminal 4: SLAM Odometry Bridge
echo "Terminal 4 - SLAM Odometry Bridge:"
echo "cd /home/aids/lunabot_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run lunabot_maintenance slam_odom_bridge"
echo ""

# Terminal 5: Static TF Publisher
echo "Terminal 5 - Static TF (map->odom):"
echo "cd /home/aids/lunabot_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0"
echo ""

# Terminal 6: Nav2 Stack
echo "Terminal 6 - Nav2 Navigation Stack:"
echo "cd /home/aids/lunabot_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true params_file:=/home/aids/lunabot_ws/src/lunabot_navigation/config/nav2_params.yaml map:=/home/aids/lunabot_ws/src/lunabot_navigation/maps/luna_map.yaml"
echo ""

# Terminal 7: RViz2 Visualization
echo "Terminal 7 - RViz2 Visualization:"
echo "cd /home/aids/lunabot_ws && unset GTK_PATH && unset LD_LIBRARY_PATH && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run rviz2 rviz2 -d /home/aids/lunabot_ws/src/lunabot_navigation/rviz/navigation.rviz"
echo ""

# Terminal 8: Frontier Explorer (NEW - Autonomous Exploration)
echo "Terminal 8 - Frontier Explorer (Autonomous Exploration):"
echo "cd /home/aids/lunabot_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run lunabot_navigation frontier_explorer"
echo ""

echo "================================================"
echo "🚨 HAZARD DETECTION (NEW - Run in additional terminals)"
echo "================================================"
echo ""
echo "Terminal 9 - Hazard Detector:"
echo "cd /home/aids/lunabot_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run lunabot_perception hazard_detector"
echo ""
echo "Terminal 10 - Control Panel (Alert Receiver):"
echo "cd /home/aids/lunabot_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run lunabot_perception control_panel"
echo ""

echo "================================================"
echo "📋 ALTERNATIVE: Fixed Waypoint Patrol (Old Method)"
echo "================================================"
echo "Terminal 8 Alternative - Autonomous Patrol (Fixed Waypoints):"
echo "cd /home/aids/lunabot_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run lunabot_navigation autonomous_patrol"
echo ""

echo "================================================"
echo "🔧 USEFUL DIAGNOSTIC COMMANDS"
echo "================================================"
echo ""
echo "Check all topics:"
echo "ros2 topic list"
echo ""
echo "Check map updates:"
echo "ros2 topic hz /map"
echo ""
echo "Check SLAM pose:"
echo "ros2 topic echo /slam_toolbox/pose"
echo ""
echo "Check odometry:"
echo "ros2 topic echo /odom"
echo ""
echo "Check scan data:"
echo "ros2 topic hz /scan"
echo ""
echo "Check Nav2 status:"
echo "ros2 node list | grep nav2"
echo ""
echo "View TF tree:"
echo "ros2 run tf2_tools view_frames"
echo ""
echo "Save current map:"
echo "ros2 run nav2_map_server map_saver_cli -f ~/habitat_map"
echo ""

echo "================================================"
echo "📦 BUILD COMMANDS"
echo "================================================"
echo ""
echo "Build all packages:"
echo "cd /home/aids/lunabot_ws && colcon build --symlink-install"
echo ""
echo "Build single package:"
echo "cd /home/aids/lunabot_ws && colcon build --packages-select lunabot_navigation --symlink-install"
echo ""
echo "Clean build:"
echo "cd /home/aids/lunabot_ws && rm -rf build install log && colcon build --symlink-install"
echo ""

echo "================================================"
echo "✅ SYSTEM READY - Run commands in separate terminals"
echo "================================================"
