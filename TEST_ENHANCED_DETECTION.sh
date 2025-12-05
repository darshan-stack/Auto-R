#!/bin/bash
# Test Enhanced Hazard Detection System

echo "═══════════════════════════════════════════════════════════════"
echo "🧪 TESTING ENHANCED HAZARD DETECTION SYSTEM"
echo "═══════════════════════════════════════════════════════════════"
echo ""

source /home/aids/lunabot_ws/install/setup.bash

echo "Step 1: Checking available topics..."
echo ""
echo "📡 LiDAR Topics:"
ros2 topic list | grep lidar
echo ""
echo "📷 Camera Topics:"
ros2 topic list | grep camera
echo ""
echo "📍 Odometry Topics:"
ros2 topic list | grep odom
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "Step 2: Testing topic availability..."
echo ""

echo "Testing 3D LiDAR..."
timeout 2 ros2 topic hz /front_3d_lidar/lidar_points 2>&1 | head -3 || echo "⚠️  LiDAR not publishing (ensure Isaac Sim is running with Play pressed)"
echo ""

echo "Testing Camera..."
timeout 2 ros2 topic hz /front_stereo_camera/left/image_raw 2>&1 | head -3 || echo "⚠️  Camera not publishing"
echo ""

echo "Testing Odometry..."
timeout 2 ros2 topic hz /chassis/odom 2>&1 | head -3 || echo "⚠️  Odometry not publishing"
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "Step 3: Checking detection parameters..."
echo ""

cat << 'EOF'
Enhanced Detection Configuration:
  • LiDAR Topic: /front_3d_lidar/lidar_points (PointCloud2)
  • Camera Topic: /front_stereo_camera/left/image_raw
  • Odometry Topic: /chassis/odom
  • Detection Range: 3.0 meters
  • Height Range: -0.2m to 2.5m
  • Minimum Points: 30 points to confirm obstacle
  • Field of View: 60 degrees

GPS Coordinate System:
  • Converts robot XY position to simulated GPS lat/lon
  • Provides absolute obstacle positions
  • Tracks both robot and obstacle locations
EOF

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "Step 4: Output directories..."
echo ""

mkdir -p ~/lunabot_hazards
echo "✅ Hazard images: ~/lunabot_hazards/"
echo "   - Annotated JPG images with obstacle details"
echo "   - Detection log: hazard_detections.json"
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "🚀 READY TO LAUNCH"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "To start enhanced hazard detection:"
echo "  ros2 run lunabot_perception enhanced_hazard_detector"
echo ""
echo "To monitor detections:"
echo "  ros2 topic echo /hazard_alerts"
echo ""
echo "To view debug images (if RViz available):"
echo "  ros2 topic echo /hazard_debug_image"
echo ""
echo "To check detection log:"
echo "  cat ~/lunabot_hazards/hazard_detections.json | jq"
echo ""
echo "To view captured images:"
echo "  ls -lht ~/lunabot_hazards/"
echo ""
