#!/bin/bash
# Quick Launch - Enhanced Hazard Detector with Fixed Topics

source /home/aids/lunabot_ws/install/setup.bash

echo "══════════════════════════════════════════════════════════════"
echo "🚨 LAUNCHING ENHANCED HAZARD DETECTION SYSTEM"
echo "══════════════════════════════════════════════════════════════"
echo ""
echo "✅ Fixed Issues:"
echo "   • Using /front_3d_lidar/lidar_points (3D PointCloud)"
echo "   • Using /chassis/odom (correct odometry)"
echo "   • Proper GPS coordinate transformation"
echo "   • Accurate obstacle positioning"
echo ""
echo "🎯 Configuration:"
echo "   • Detection Range: 3.0 meters"
echo "   • Height Range: -0.2m to 2.5m"
echo "   • Min Points: 30 (sensitive detection)"
echo "   • Cooldown: 30 iterations"
echo ""
echo "📁 Output:"
echo "   • Images: ~/lunabot_hazards/*.jpg"
echo "   • Log: ~/lunabot_hazards/hazard_detections.json"
echo "   • Alerts: /hazard_alerts topic"
echo ""
echo "══════════════════════════════════════════════════════════════"
echo ""
echo "Starting enhanced hazard detector..."
echo ""

ros2 run lunabot_perception enhanced_hazard_detector
