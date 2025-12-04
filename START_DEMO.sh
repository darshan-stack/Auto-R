#!/bin/bash

echo "======================================"
echo "ISRO LunaBot Autonomous Navigation Demo"
echo "======================================"
echo ""

cd /home/aids/lunabot_ws

# Source ROS 2
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Step 1: Starting Environmental Monitor..."
gnome-terminal --tab --title="Environment Monitor" -- bash -c "
cd /home/aids/lunabot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lunabot_maintenance env_monitor
exec bash"

sleep 2

echo "Step 2: Starting PointCloud to LaserScan Converter..."
gnome-terminal --tab --title="Sensor Bridge" -- bash -c "
cd /home/aids/lunabot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lunabot_navigation pointcloud_to_scan --ros-args -p use_sim_time:=true
exec bash"

sleep 2

echo "Step 3: Starting SLAM (Mapping)..."
gnome-terminal --tab --title="SLAM" -- bash -c "
cd /home/aids/lunabot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
exec bash"

sleep 3

echo "Step 4: Starting Navigation Stack..."
gnome-terminal --tab --title="Navigation" -- bash -c "
cd /home/aids/lunabot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=\$(pwd)/src/lunabot_navigation/config/nav2_params.yaml
exec bash"

sleep 5

echo "Step 5: Starting RViz..."
gnome-terminal --tab --title="RViz" -- bash -c "
cd /home/aids/lunabot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rviz2 rviz2 -d src/lunabot_navigation/rviz/navigation.rviz
exec bash"

sleep 3

echo "Step 6: Starting Autonomous Patrol..."
gnome-terminal --tab --title="Autonomous Patrol" -- bash -c "
cd /home/aids/lunabot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
echo 'Waiting 10 seconds for navigation to initialize...'
sleep 10
ros2 run lunabot_maintenance autonomous_patrol
exec bash"

echo ""
echo "======================================"
echo "All systems launched!"
echo "======================================"
echo ""
echo "What you should see:"
echo "  • Environment Monitor: Temperature, O2, Pressure monitoring"
echo "  • SLAM: Building map in real-time"
echo "  • Navigation: Path planning and obstacle avoidance"
echo "  • RViz: Visualization of map, robot, costmaps, paths"
echo "  • Autonomous Patrol: Robot moving to waypoints automatically"
echo ""
echo "Press Ctrl+C in any terminal to stop that component"
echo ""
