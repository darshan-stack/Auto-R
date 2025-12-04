#!/bin/bash
# Quick verification script for all 6 running components

echo "═══════════════════════════════════════════════════════════════"
echo "🔍 ISRO LUNABOT - SYSTEM VERIFICATION"
echo "═══════════════════════════════════════════════════════════════"
echo ""

# Source ROS 2
source /home/aids/lunabot_ws/install/setup.bash

echo "1️⃣  CHECKING ACTIVE NODES..."
echo "─────────────────────────────────────────────────────────────"
ros2 node list | grep -E "(environment_monitor|scan_relay|slam|hazard|control|frontier)" || echo "⚠️  Some nodes missing"
echo ""

echo "2️⃣  CHECKING CRITICAL TOPICS..."
echo "─────────────────────────────────────────────────────────────"
ros2 topic list | grep -E "(hazard_alerts|habitat|scan|odom|map)" || echo "⚠️  Some topics missing"
echo ""

echo "3️⃣  ENVIRONMENTAL MONITOR STATUS..."
echo "─────────────────────────────────────────────────────────────"
timeout 2 ros2 topic echo /habitat/temperature --once 2>/dev/null && echo "✅ Temperature topic active" || echo "⚠️  Temperature topic not available"
timeout 2 ros2 topic echo /habitat/pressure --once 2>/dev/null && echo "✅ Pressure topic active" || echo "⚠️  Pressure topic not available"
timeout 2 ros2 topic echo /habitat/oxygen --once 2>/dev/null && echo "✅ Oxygen topic active" || echo "⚠️  Oxygen topic not available"
echo ""

echo "4️⃣  HAZARD DETECTION STATUS..."
echo "─────────────────────────────────────────────────────────────"
ros2 topic info /hazard_alerts 2>/dev/null && echo "✅ Hazard alerts topic active" || echo "⚠️  Hazard alerts not available"
ros2 topic info /hazard_stop 2>/dev/null && echo "✅ Hazard stop topic active" || echo "⚠️  Hazard stop not available"
echo ""

echo "5️⃣  CHECKING HAZARD IMAGE DIRECTORY..."
echo "─────────────────────────────────────────────────────────────"
if [ -d ~/lunabot_hazards ]; then
    IMAGE_COUNT=$(ls -1 ~/lunabot_hazards/*.jpg 2>/dev/null | wc -l)
    echo "✅ Hazard directory exists"
    echo "📸 Total hazard images captured: $IMAGE_COUNT"
    if [ $IMAGE_COUNT -gt 0 ]; then
        echo "Latest images:"
        ls -lht ~/lunabot_hazards/*.jpg 2>/dev/null | head -3
    fi
else
    echo "⚠️  Hazard directory not found (will be created on first detection)"
fi
echo ""

echo "6️⃣  CHECKING CONTROL PANEL LOG..."
echo "─────────────────────────────────────────────────────────────"
if [ -f ~/lunabot_control_panel/hazard_log.txt ]; then
    ALERT_COUNT=$(wc -l < ~/lunabot_control_panel/hazard_log.txt 2>/dev/null)
    echo "✅ Control panel log exists"
    echo "📋 Total alerts logged: $ALERT_COUNT lines"
    if [ $ALERT_COUNT -gt 0 ]; then
        echo "Latest alerts:"
        tail -5 ~/lunabot_control_panel/hazard_log.txt
    fi
else
    echo "⚠️  Control panel log not found (will be created on first alert)"
fi
echo ""

echo "7️⃣  SLAM MAP STATUS..."
echo "─────────────────────────────────────────────────────────────"
timeout 2 ros2 topic echo /map --once 2>/dev/null | head -10 && echo "✅ Map topic active" || echo "⚠️  Map not available yet"
echo ""

echo "8️⃣  NAVIGATION STATUS..."
echo "─────────────────────────────────────────────────────────────"
timeout 2 ros2 topic echo /odom --once 2>/dev/null | head -10 && echo "✅ Odometry active" || echo "⚠️  Odometry not available"
echo ""

echo "═══════════════════════════════════════════════════════════════"
echo "✅ VERIFICATION COMPLETE"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "📸 NOW CAPTURE SCREENSHOTS FROM EACH TERMINAL:"
echo ""
echo "Terminal 1: Environmental Monitor (b44b4688...)"
echo "Terminal 2: Scan Relay (cbe2b4e9...)"
echo "Terminal 3: SLAM Toolbox (70b882a4...)"
echo "Terminal 4: Odometry Bridge (c0e117b7...)"
echo "Terminal 5: Nav2 Stack (b0bc1ec1...)"
echo "Terminal 6: Hazard Detector (fe9bac96...)"
echo "Terminal 7: Control Panel (0f6054fb...)"
echo "Terminal 8: Frontier Explorer (3f56581d...)"
echo ""
echo "Save all captures to: ~/lunabot_demo_captures/"
echo ""
