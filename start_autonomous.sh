#!/bin/bash
# ISRO LunaBot Full Autonomous System Launcher
# Industry-Ready Implementation

echo "================================================================"
echo "  🌙 ISRO LUNABOT CHALLENGE - AUTONOMOUS NAVIGATION SYSTEM"
echo "  Problem Statement #25169: Autonomous Navigation for Lunar Habitats"
echo "================================================================"
echo ""
echo "System Components:"
echo "  ✅ Environmental Monitoring (Temp, O2, Pressure)"
echo "  ✅ 3D LiDAR to 2D Scan Conversion"
echo "  ✅ Nav2 Navigation Stack (AMCL + DWB + NavFn)"
echo "  ✅ Autonomous Waypoint Patrol"
echo "  ✅ RViz2 Visualization"
echo "  ✅ Isaac Sim Integration"
echo ""
echo "================================================================"
echo ""

# Check if Isaac Sim is running
echo "⚠️  IMPORTANT: Make sure Isaac Sim is running with Play button pressed!"
read -p "Press Enter when Isaac Sim is ready..."

echo ""
echo "🚀 Launching Full Autonomous System..."
echo ""

# Source workspace
cd /home/aids/lunabot_ws
source install/setup.bash

# Launch the complete system
ros2 launch lunabot_navigation full_autonomous.launch.py

echo ""
echo "================================================================"
echo "  System Shutdown Complete"
echo "================================================================"
