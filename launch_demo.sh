#!/bin/bash
# Complete launch script for LunaBot ISRO Challenge Demo

echo "======================================"
echo "  LunaBot ISRO Challenge Demo Launch"
echo "======================================"
echo ""
echo "Prerequisites:"
echo "  ✓ Isaac Sim running with nova_carter_ROS"
echo "  ✓ Press PLAY in Isaac Sim"
echo ""
read -p "Press ENTER when Isaac Sim is ready..."

cd /home/aids/lunabot_ws
source install/setup.bash
export AMENT_PREFIX_PATH="/home/aids/lunabot_ws/install/lunabot_navigation:$AMENT_PREFIX_PATH"

echo ""
echo "Starting components..."
echo ""

# Launch environmental monitoring in background
gnome-terminal --tab --title="Environment Monitor" -- bash -c "
    source /home/aids/lunabot_ws/install/setup.bash;
    echo '🌡️  Starting Environment Monitor...';
    ros2 run lunabot_maintenance env_monitor;
    exec bash
" &

sleep 2

# Launch navigation system
gnome-terminal --tab --title="Navigation System" -- bash -c "
    source /home/aids/lunabot_ws/install/setup.bash;
    export AMENT_PREFIX_PATH='/home/aids/lunabot_ws/install/lunabot_navigation:\$AMENT_PREFIX_PATH';
    echo '🤖 Starting Navigation Stack...';
    ros2 launch lunabot_navigation nova_carter_nav.launch.py;
    exec bash
" &

echo ""
echo "✅ All systems starting..."
echo ""
echo "Available commands:"
echo "  - Check topics: ros2 topic list"
echo "  - Monitor environment: ros2 topic echo /habitat/system_status"
echo "  - Send navigation goal:"
echo "    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\"
echo "      \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}\""
echo ""
echo "Press Ctrl+C to stop all systems"
echo ""

wait
