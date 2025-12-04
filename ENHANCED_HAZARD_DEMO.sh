#!/bin/bash
# Enhanced Hazard Detection Demo with Path Tracking
# Shows obstacle detection, image capture, path replanning, and footprint recording

echo "═══════════════════════════════════════════════════════════════"
echo "🚨 ENHANCED HAZARD DETECTION & PATH TRACKING DEMO"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "This demo showcases:"
echo "  ✅ Real-time hazard detection (LiDAR + Visual)"
echo "  ✅ Automatic image capture with GPS coordinates"
echo "  ✅ Intelligent path replanning around obstacles"
echo "  ✅ Control panel alerts with hazard details"
echo "  ✅ Path footprint recording with coordinates"
echo "  ✅ Complete traversal history logging"
echo ""
echo "═══════════════════════════════════════════════════════════════"
echo ""

# Source workspace
source /home/aids/lunabot_ws/install/setup.bash

# Create output directories
mkdir -p ~/lunabot_hazards
mkdir -p ~/lunabot_path_logs
mkdir -p ~/lunabot_control_panel

echo "📁 Output Directories Created:"
echo "   Hazard Images: ~/lunabot_hazards/"
echo "   Path Logs: ~/lunabot_path_logs/"
echo "   Control Panel: ~/lunabot_control_panel/"
echo ""

# Function to launch in new terminal (for demonstration)
launch_node() {
    echo "▶️  Launching: $1"
    echo "   Command: $2"
    echo ""
}

echo "═══════════════════════════════════════════════════════════════"
echo "📋 STEP-BY-STEP LAUNCH SEQUENCE"
echo "═══════════════════════════════════════════════════════════════"
echo ""

echo "STEP 1: Environmental Monitor"
launch_node "Environmental Monitoring" "ros2 run lunabot_maintenance env_monitor"

echo "STEP 2: Scan Relay"
launch_node "LiDAR Scan Relay" "ros2 run lunabot_maintenance scan_relay"

echo "STEP 3: SLAM Toolbox"
launch_node "SLAM Mapping" "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/aids/lunabot_ws/src/lunabot_navigation/config/slam_params.yaml use_sim_time:=true"

echo "STEP 4: Odometry Bridge"
launch_node "Odometry Bridge" "ros2 run lunabot_maintenance slam_odom_bridge"

echo "STEP 5: Static TF"
launch_node "TF Publisher" "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom"

echo "STEP 6: Nav2 Navigation"
launch_node "Nav2 Stack" "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=/home/aids/lunabot_ws/src/lunabot_navigation/config/nav2_params.yaml"

echo "STEP 7: Path Tracker (NEW!)"
launch_node "Path Footprint Recorder" "ros2 run lunabot_navigation path_tracker"

echo "STEP 8: Enhanced Hazard Detector"
launch_node "Hazard Detection System" "ros2 run lunabot_perception hazard_detector"

echo "STEP 9: Control Panel"
launch_node "Control Panel Receiver" "ros2 run lunabot_perception control_panel"

echo "STEP 10: Frontier Explorer"
launch_node "Autonomous Exploration" "ros2 run lunabot_navigation frontier_explorer"

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "📊 MONITORING & VERIFICATION COMMANDS"
echo "═══════════════════════════════════════════════════════════════"
echo ""

cat << 'EOF'
# Check hazard detections in real-time
ros2 topic echo /hazard_alerts

# Monitor path footprint
ros2 topic echo /path_footprint

# View traveled path (for RViz)
ros2 topic echo /traveled_path

# Check all active topics
ros2 topic list | grep -E "(hazard|path|footprint)"

# View hazard images
ls -lht ~/lunabot_hazards/

# View path logs
ls -lht ~/lunabot_path_logs/

# View latest path CSV
tail -20 ~/lunabot_path_logs/path_*.csv

# View hazard detection log
cat ~/lunabot_hazards/hazard_detections.json

# View control panel log
cat ~/lunabot_control_panel/hazard_log.txt

EOF

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "🎯 TESTING HAZARD DETECTION"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "To trigger hazard detection:"
echo "  1. Place obstacle in robot's path in Isaac Sim"
echo "  2. Watch terminal for hazard alerts"
echo "  3. Check ~/lunabot_hazards/ for captured images"
echo "  4. View control panel for alert details"
echo "  5. Observe Nav2 replanning path around obstacle"
echo "  6. Check path logs for coordinate history"
echo ""
echo "Expected behavior:"
echo "  🔴 Robot detects obstacle at <2.0m distance"
echo "  📸 Image captured with annotations"
echo "  🛑 Robot stops immediately"
echo "  📡 Alert sent to control panel with GPS coordinates"
echo "  🔄 Nav2 automatically replans path around obstacle"
echo "  📍 New path recorded in footprint log"
echo "  ✅ Robot continues exploration"
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "📸 OUTPUT FILES TO CAPTURE FOR DEMO"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "1. Hazard Image Example:"
echo "   ~/lunabot_hazards/hazard_0001_YYYYMMDD_HHMMSS.jpg"
echo "   (Shows obstacle with distance, GPS, timestamp overlay)"
echo ""
echo "2. Hazard Detection Log:"
echo "   ~/lunabot_hazards/hazard_detections.json"
echo "   (JSON with all detection details)"
echo ""
echo "3. Path Footprint CSV:"
echo "   ~/lunabot_path_logs/path_YYYYMMDD_HHMMSS.csv"
echo "   (X,Y,Z coordinates with timestamps)"
echo ""
echo "4. Path Footprint JSON:"
echo "   ~/lunabot_path_logs/path_YYYYMMDD_HHMMSS.json"
echo "   (Complete path with all waypoints)"
echo ""
echo "5. Control Panel Log:"
echo "   ~/lunabot_control_panel/hazard_log.txt"
echo "   (Alert log with formatted output)"
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "✅ SETUP COMPLETE - READY TO LAUNCH"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "Launch each step in a separate terminal in order."
echo "Monitor outputs and capture screenshots for demo."
echo ""
