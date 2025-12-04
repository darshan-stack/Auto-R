#!/bin/bash
# Quick Start - Enhanced Hazard Detection Demo
# Launches path tracker and enhanced hazard detector

source /home/aids/lunabot_ws/install/setup.bash

echo "═══════════════════════════════════════════════════════════════"
echo "🚀 QUICK START: Enhanced Hazard Detection with Path Tracking"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "Launching NEW components:"
echo "  ✅ Path Tracker - Records complete robot trajectory"
echo "  ✅ Enhanced Hazard Detector - Improved detection + path replanning"
echo ""
echo "Prerequisites (already running):"
echo "  • Environmental Monitor"
echo "  • Scan Relay  "
echo "  • SLAM Toolbox"
echo "  • Odometry Bridge"
echo "  • Static TF"
echo "  • Nav2 Stack"
echo "  • Control Panel"
echo "  • Frontier Explorer"
echo ""
echo "═══════════════════════════════════════════════════════════════"
echo ""

# Check if output directories exist
mkdir -p ~/lunabot_hazards
mkdir -p ~/lunabot_path_logs
mkdir -p ~/lunabot_control_panel

echo "📁 Output directories ready:"
echo "   ~/lunabot_hazards/ (hazard images + logs)"
echo "   ~/lunabot_path_logs/ (path CSV + JSON)"
echo "   ~/lunabot_control_panel/ (alert logs)"
echo ""

echo "Press Enter to start Path Tracker..."
read

echo "Starting Path Tracker..."
ros2 run lunabot_navigation path_tracker &
PATH_TRACKER_PID=$!
sleep 2

echo ""
echo "Press Enter to start Enhanced Hazard Detector..."
read

echo "Starting Enhanced Hazard Detector..."
ros2 run lunabot_perception hazard_detector &
HAZARD_PID=$!

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "✅ BOTH NODES RUNNING"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "Path Tracker PID: $PATH_TRACKER_PID"
echo "Hazard Detector PID: $HAZARD_PID"
echo ""
echo "Monitoring commands:"
echo "  ros2 topic echo /hazard_alerts"
echo "  ros2 topic echo /path_footprint"
echo "  ros2 topic echo /traveled_path"
echo ""
echo "File monitoring:"
echo "  watch ls -lh ~/lunabot_hazards/"
echo "  tail -f ~/lunabot_path_logs/path_*.csv"
echo ""
echo "Press Ctrl+C to stop all nodes..."
echo ""

# Wait for user interrupt
wait
