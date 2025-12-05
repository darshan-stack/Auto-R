#!/bin/bash
# View Hazard Detection JSON Log

JSON_FILE=~/lunabot_hazards/hazard_detections.json

if [ ! -f "$JSON_FILE" ]; then
    echo "❌ No hazard log found at: $JSON_FILE"
    echo "💡 Run the enhanced detector first to generate detections"
    exit 1
fi

echo "══════════════════════════════════════════════════════════════"
echo "📋 HAZARD DETECTION LOG VIEWER"
echo "══════════════════════════════════════════════════════════════"
echo ""

# Display summary
echo "📊 SUMMARY:"
cat "$JSON_FILE" | jq -r '"Total Detections: \(.total_detections)"'
cat "$JSON_FILE" | jq -r '"Session Start: \(.session_start)"'
echo ""

# Display each detection
echo "🚨 DETECTIONS:"
echo "══════════════════════════════════════════════════════════════"

cat "$JSON_FILE" | jq -r '
.detections[] | 
"
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🔍 Detection #\(.detection_id)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📅 Timestamp: \(.timestamp)
📏 Distance: \(.distance_meters)m
🎯 Points: \(.obstacle_info.num_points)
📷 Image: \(.image_filename)

🤖 ROBOT POSITION:
   X: \(.robot_position.x)
   Y: \(.robot_position.y)
   Z: \(.robot_position.z)
   Yaw: \(.robot_position.yaw_radians) rad

🧱 OBSTACLE ABSOLUTE POSITION:
   X: \(.obstacle_absolute_position.x)
   Y: \(.obstacle_absolute_position.y)
   Z: \(.obstacle_absolute_position.z)

🌍 GPS COORDINATES:
   Latitude: \(.gps_coordinates.latitude)
   Longitude: \(.gps_coordinates.longitude)
   Altitude: \(.gps_coordinates.altitude)

📍 OBSTACLE RELATIVE POSITION:
   X: \(.obstacle_relative_position.x)m (forward)
   Y: \(.obstacle_relative_position.y)m (left/right)
   Z: \(.obstacle_relative_position.z)m (height)
"
'

echo "══════════════════════════════════════════════════════════════"
echo ""
echo "📂 Full log: $JSON_FILE"
echo "📸 Images: ~/lunabot_hazards/*.jpg"
echo ""
echo "💡 TIPS:"
echo "   View raw JSON: cat $JSON_FILE | jq"
echo "   Count detections: cat $JSON_FILE | jq '.total_detections'"
echo "   List images: ls -lht ~/lunabot_hazards/*.jpg"
echo "   Latest detection: cat $JSON_FILE | jq '.detections[-1]'"
echo ""
