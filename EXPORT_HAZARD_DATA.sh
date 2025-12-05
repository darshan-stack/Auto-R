#!/bin/bash
# Export Hazard Detection Data in Multiple Formats

JSON_FILE=~/lunabot_hazards/hazard_detections.json
OUTPUT_DIR=~/lunabot_hazards/exports
mkdir -p "$OUTPUT_DIR"

if [ ! -f "$JSON_FILE" ]; then
    echo "❌ No hazard log found"
    exit 1
fi

echo "══════════════════════════════════════════════════════════════"
echo "📤 EXPORTING HAZARD DETECTION DATA"
echo "══════════════════════════════════════════════════════════════"
echo ""

# 1. Export to CSV
echo "📊 Exporting to CSV..."
CSV_FILE="$OUTPUT_DIR/hazard_detections.csv"
cat "$JSON_FILE" | jq -r '
["Detection_ID", "Timestamp", "Distance_m", "Points", "Robot_X", "Robot_Y", "Robot_Z", "Obstacle_X", "Obstacle_Y", "Obstacle_Z", "GPS_Lat", "GPS_Lon", "Image"],
(.detections[] | [
  .detection_id,
  .timestamp,
  .distance_meters,
  .obstacle_info.num_points,
  .robot_position.x,
  .robot_position.y,
  .robot_position.z,
  .obstacle_absolute_position.x,
  .obstacle_absolute_position.y,
  .obstacle_absolute_position.z,
  .gps_coordinates.latitude,
  .gps_coordinates.longitude,
  .image_filename
]) | @csv
' > "$CSV_FILE"
echo "   ✅ Saved: $CSV_FILE"

# 2. Export GPS waypoints
echo "🌍 Exporting GPS waypoints..."
GPS_FILE="$OUTPUT_DIR/hazard_gps_waypoints.txt"
cat "$JSON_FILE" | jq -r '
.detections[] | 
"\(.gps_coordinates.latitude),\(.gps_coordinates.longitude),\(.gps_coordinates.altitude)"
' > "$GPS_FILE"
echo "   ✅ Saved: $GPS_FILE"

# 3. Export obstacle positions only
echo "📍 Exporting obstacle positions..."
POS_FILE="$OUTPUT_DIR/obstacle_positions.txt"
cat "$JSON_FILE" | jq -r '
.detections[] | 
"ID: \(.detection_id) | X: \(.obstacle_absolute_position.x) | Y: \(.obstacle_absolute_position.y) | Z: \(.obstacle_absolute_position.z)"
' > "$POS_FILE"
echo "   ✅ Saved: $POS_FILE"

# 4. Create summary report
echo "📋 Creating summary report..."
SUMMARY_FILE="$OUTPUT_DIR/detection_summary.txt"
cat > "$SUMMARY_FILE" << EOF
HAZARD DETECTION SUMMARY REPORT
Generated: $(date)
════════════════════════════════════════════════════════════════

STATISTICS:
$(cat "$JSON_FILE" | jq -r '"  Total Detections: \(.total_detections)"')
$(cat "$JSON_FILE" | jq -r '"  Session Start: \(.session_start)"')

DETECTION DISTANCES:
$(cat "$JSON_FILE" | jq -r '.detections[] | "  #\(.detection_id): \(.distance_meters)m"')

AVERAGE STATISTICS:
$(cat "$JSON_FILE" | jq -r '
  (.detections | map(.distance_meters) | add / length) as $avg_dist |
  (.detections | map(.obstacle_info.num_points) | add / length) as $avg_points |
  "  Average Distance: \($avg_dist)m\n  Average Points: \($avg_points)"
')

FILES:
  JSON: $JSON_FILE
  CSV: $CSV_FILE
  GPS: $GPS_FILE
  Positions: $POS_FILE
  Images: ~/lunabot_hazards/*.jpg

════════════════════════════════════════════════════════════════
EOF
echo "   ✅ Saved: $SUMMARY_FILE"

# 5. Display exports
echo ""
echo "══════════════════════════════════════════════════════════════"
echo "✅ EXPORT COMPLETE"
echo "══════════════════════════════════════════════════════════════"
echo ""
echo "📁 Exported files:"
ls -lh "$OUTPUT_DIR"
echo ""
echo "📋 Summary:"
cat "$SUMMARY_FILE"
