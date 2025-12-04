#!/bin/bash

echo "══════════════════════════════════════════════════════"
echo "  SIMPLIFIED DEMO - Works WITHOUT Isaac Sim Odometry"
echo "══════════════════════════════════════════════════════"
echo ""
echo "⚠️  Isaac Sim isn't publishing odometry"
echo "✅ Solution: Using SLAM pose as odometry instead"
echo ""
echo "This demo will:"
echo "  • Use SLAM for mapping AND odometry"
echo "  • Let you drive the robot with keyboard"
echo "  • Build a map as you drive"
echo "  • Then navigate autonomously using that map"
echo ""
echo "══════════════════════════════════════════════════════"
echo ""

cd /home/aids/lunabot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Check if processes are already running
if pgrep -f "slam_toolbox" > /dev/null; then
    echo "✅ SLAM is already running"
else
    echo "❌ SLAM not running - please start it first!"
    echo ""
    echo "Run this in another terminal:"
    echo "cd /home/aids/lunabot_ws"
    echo "source /opt/ros/humble/setup.bash"
    echo "source install/setup.bash"
    echo "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true params_file:=/home/aids/lunabot_ws/src/lunabot_navigation/config/slam_params.yaml"
    echo ""
    exit 1
fi

echo "══════════════════════════════════════════════════════"
echo "  Starting SLAM Odometry Bridge"
echo "══════════════════════════════════════════════════════"
ros2 run lunabot_maintenance slam_odom_bridge &
BRIDGE_PID=$!
sleep 2

echo "══════════════════════════════════════════════════════"
echo "  KEYBOARD TELEOP CONTROLS"
echo "══════════════════════════════════════════════════════"
echo ""
echo "Drive the robot around to build a map:"
echo "   i = forward"
echo "   , = backward"
echo "   j = turn left"
echo "   l = turn right"
echo "   k = stop"
echo ""
echo "Drive for 30-60 seconds to build a good map"
echo "Then press Ctrl+C to stop and move to navigation"
echo ""
echo "══════════════════════════════════════════════════════"
echo ""

ros2 run lunabot_maintenance teleop_keyboard

echo ""
echo "══════════════════════════════════════════════════════"
echo "  Map built! Ready for autonomous navigation"
echo "══════════════════════════════════════════════════════"
echo ""
echo "Stopping SLAM odometry bridge..."
kill $BRIDGE_PID

echo "✅ Now you can run navigation and autonomous patrol"
