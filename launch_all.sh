#!/bin/bash
# Complete Autonomous System Launcher
# Bypasses package registration issues

cd /home/aids/lunabot_ws

# Setup environment manually
export AMENT_PREFIX_PATH=/home/aids/lunabot_ws/install/lunabot_navigation:/home/aids/lunabot_ws/install/lunabot_maintenance:/home/aids/lunabot_ws/install/lunar_bot_pkg:/home/aids/lunabot_ws/install/lunabot_perception:/opt/ros/humble
export PYTHONPATH=/home/aids/lunabot_ws/install/lunabot_navigation/lib/python3.10/site-packages:/home/aids/lunabot_ws/install/lunabot_maintenance/lib/python3.10/site-packages:$PYTHONPATH
export PATH=/opt/ros/humble/bin:$PATH
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH

echo "=========================================================="
echo "  🌙 ISRO LUNABOT - FULL AUTONOMOUS SYSTEM"
echo "=========================================================="
echo ""
echo "Starting all components in background..."
echo ""

# 1. Environmental Monitor
echo "✅ Starting Environmental Monitor..."
ros2 run lunabot_maintenance env_monitor --ros-args -p use_sim_time:=true &
sleep 2

# 2. Static TF Publisher (map -> odom)
echo "✅ Starting Static TF Publisher..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom --ros-args -p use_sim_time:=true &
sleep 1

# 3. PointCloud to LaserScan Converter
echo "✅ Starting LiDAR Converter..."
ros2 run lunabot_navigation pointcloud_to_scan --ros-args -p use_sim_time:=true &
sleep 2

# 4. Nav2 Bringup
echo "✅ Starting Nav2 Navigation Stack..."
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=true \
    map:=/home/aids/lunabot_ws/install/lunabot_navigation/share/lunabot_navigation/maps/luna_map.yaml \
    params_file:=/home/aids/lunabot_ws/install/lunabot_navigation/share/lunabot_navigation/config/nav2_params.yaml \
    autostart:=true &
NAV_PID=$!
sleep 10

# 5. RViz
echo "✅ Starting RViz..."
rviz2 -d /home/aids/lunabot_ws/install/lunabot_navigation/share/lunabot_navigation/rviz/navigation.rviz --ros-args -p use_sim_time:=true &
sleep 3

# 6. Autonomous Patrol
echo "✅ Starting Autonomous Patrol..."
echo ""
echo "=========================================================="
echo "  🚀 ROBOT WILL START MOVING AUTONOMOUSLY IN 15 SECONDS"
echo "=========================================================="
sleep 5
ros2 run lunabot_navigation autonomous_patrol --ros-args -p use_sim_time:=true

# Keep script running
wait $NAV_PID
