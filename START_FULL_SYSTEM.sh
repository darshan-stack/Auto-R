#!/bin/bash
# MASTER LAUNCHER - Complete Lunabot System (10 Components)
# Run this after pressing Play in Isaac Sim

echo "════════════════════════════════════════════════════════════════════════════"
echo "🚀 LUNABOT COMPLETE SYSTEM STARTUP - 10 COMPONENTS"
echo "════════════════════════════════════════════════════════════════════════════"
echo ""
echo "⚠️  PREREQUISITES:"
echo "   1. Isaac Sim is running"
echo "   2. Nova Carter robot loaded"
echo "   3. ▶️  PLAY BUTTON PRESSED IN ISAAC SIM"
echo ""
echo "════════════════════════════════════════════════════════════════════════════"
echo ""

# Check if Isaac Sim topics are publishing
echo "🔍 Checking Isaac Sim connection..."
timeout 3 ros2 topic hz /front_3d_lidar/lidar_points --once > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "   ✅ LiDAR detected"
else
    echo "   ⚠️  LiDAR not publishing - Is Play button pressed?"
fi

timeout 2 ros2 topic hz /chassis/odom --once > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "   ✅ Odometry detected"
else
    echo "   ⚠️  Odometry not publishing - Is Play button pressed?"
fi

echo ""
echo "════════════════════════════════════════════════════════════════════════════"
echo "🎯 LAUNCHING 10-COMPONENT SYSTEM IN TMUX"
echo "════════════════════════════════════════════════════════════════════════════"
echo ""

# Kill existing session
tmux kill-session -t lunabot 2>/dev/null

# Create tmux session
tmux new-session -d -s lunabot -n "control"

# Component 1: Environment Monitor
echo "📊 [1/10] Launching Environment Monitor..."
tmux new-window -t lunabot:1 -n "env_monitor"
tmux send-keys -t lunabot:1 "cd /home/aids/lunabot_ws" C-m
tmux send-keys -t lunabot:1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t lunabot:1 "source install/setup.bash" C-m
tmux send-keys -t lunabot:1 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:1 "echo '📊 ENVIRONMENT MONITOR'" C-m
tmux send-keys -t lunabot:1 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:1 "ros2 run lunabot_maintenance env_monitor" C-m
sleep 2

# Component 2: Scan Relay
echo "📡 [2/10] Launching Scan Relay..."
tmux new-window -t lunabot:2 -n "scan_relay"
tmux send-keys -t lunabot:2 "cd /home/aids/lunabot_ws" C-m
tmux send-keys -t lunabot:2 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t lunabot:2 "source install/setup.bash" C-m
tmux send-keys -t lunabot:2 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:2 "echo '📡 SCAN RELAY'" C-m
tmux send-keys -t lunabot:2 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:2 "ros2 run lunabot_maintenance scan_relay" C-m
sleep 2

# Component 3: SLAM Toolbox
echo "🗺️  [3/10] Launching SLAM Toolbox..."
tmux new-window -t lunabot:3 -n "slam"
tmux send-keys -t lunabot:3 "cd /home/aids/lunabot_ws" C-m
tmux send-keys -t lunabot:3 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t lunabot:3 "source install/setup.bash" C-m
tmux send-keys -t lunabot:3 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:3 "echo '🗺️  SLAM TOOLBOX'" C-m
tmux send-keys -t lunabot:3 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:3 "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/aids/lunabot_ws/src/lunabot_navigation/config/slam_params.yaml use_sim_time:=true" C-m
sleep 3

# Component 4: SLAM Odometry Bridge
echo "🔗 [4/10] Launching SLAM Odometry Bridge..."
tmux new-window -t lunabot:4 -n "slam_odom"
tmux send-keys -t lunabot:4 "cd /home/aids/lunabot_ws" C-m
tmux send-keys -t lunabot:4 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t lunabot:4 "source install/setup.bash" C-m
tmux send-keys -t lunabot:4 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:4 "echo '🔗 SLAM ODOMETRY BRIDGE'" C-m
tmux send-keys -t lunabot:4 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:4 "ros2 run lunabot_maintenance slam_odom_bridge" C-m
sleep 2

# Component 5: Static TF Publisher
echo "📍 [5/10] Launching Static TF Publisher..."
tmux new-window -t lunabot:5 -n "static_tf"
tmux send-keys -t lunabot:5 "cd /home/aids/lunabot_ws" C-m
tmux send-keys -t lunabot:5 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t lunabot:5 "source install/setup.bash" C-m
tmux send-keys -t lunabot:5 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:5 "echo '📍 STATIC TF: map -> odom'" C-m
tmux send-keys -t lunabot:5 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:5 "ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0" C-m
sleep 2

# Component 6: Nav2 Navigation Stack
echo "🧭 [6/10] Launching Nav2 Navigation Stack..."
tmux new-window -t lunabot:6 -n "nav2"
tmux send-keys -t lunabot:6 "cd /home/aids/lunabot_ws" C-m
tmux send-keys -t lunabot:6 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t lunabot:6 "source install/setup.bash" C-m
tmux send-keys -t lunabot:6 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:6 "echo '🧭 NAV2 NAVIGATION STACK'" C-m
tmux send-keys -t lunabot:6 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:6 "ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true params_file:=/home/aids/lunabot_ws/src/lunabot_navigation/config/nav2_params.yaml map:=/home/aids/lunabot_ws/src/lunabot_navigation/maps/luna_map.yaml" C-m
sleep 5

# Component 7: RViz2 Visualization
echo "👁️  [7/10] Launching RViz2 Visualization..."
tmux new-window -t lunabot:7 -n "rviz"
tmux send-keys -t lunabot:7 "cd /home/aids/lunabot_ws" C-m
tmux send-keys -t lunabot:7 "unset GTK_PATH" C-m
tmux send-keys -t lunabot:7 "unset LD_LIBRARY_PATH" C-m
tmux send-keys -t lunabot:7 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t lunabot:7 "source install/setup.bash" C-m
tmux send-keys -t lunabot:7 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:7 "echo '👁️  RVIZ2 VISUALIZATION'" C-m
tmux send-keys -t lunabot:7 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:7 "ros2 run rviz2 rviz2 -d /home/aids/lunabot_ws/src/lunabot_navigation/rviz/navigation.rviz" C-m
sleep 3

# Component 8: Frontier Explorer
echo "🔍 [8/10] Launching Frontier Explorer..."
tmux new-window -t lunabot:8 -n "frontier"
tmux send-keys -t lunabot:8 "cd /home/aids/lunabot_ws" C-m
tmux send-keys -t lunabot:8 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t lunabot:8 "source install/setup.bash" C-m
tmux send-keys -t lunabot:8 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:8 "echo '🔍 FRONTIER EXPLORER'" C-m
tmux send-keys -t lunabot:8 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:8 "ros2 run lunabot_navigation frontier_explorer" C-m
sleep 2

# Component 9: Path Tracker
echo "📊 [9/10] Launching Path Tracker..."
tmux new-window -t lunabot:9 -n "path_tracker"
tmux send-keys -t lunabot:9 "cd /home/aids/lunabot_ws" C-m
tmux send-keys -t lunabot:9 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t lunabot:9 "source install/setup.bash" C-m
tmux send-keys -t lunabot:9 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:9 "echo '📊 PATH TRACKER'" C-m
tmux send-keys -t lunabot:9 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:9 "ros2 run lunabot_navigation path_tracker" C-m
sleep 2

# Component 10: Enhanced Hazard Detection
echo "🚨 [10/10] Launching Enhanced Hazard Detection..."
tmux new-window -t lunabot:10 -n "hazard"
tmux send-keys -t lunabot:10 "cd /home/aids/lunabot_ws" C-m
tmux send-keys -t lunabot:10 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t lunabot:10 "source install/setup.bash" C-m
tmux send-keys -t lunabot:10 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:10 "echo '🚨 ENHANCED HAZARD DETECTION'" C-m
tmux send-keys -t lunabot:10 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:10 "echo '✅ LiDAR: /front_3d_lidar/lidar_points'" C-m
tmux send-keys -t lunabot:10 "echo '✅ Odom: /chassis/odom'" C-m
tmux send-keys -t lunabot:10 "echo '✅ GPS transformation enabled'" C-m
tmux send-keys -t lunabot:10 "echo '✅ JSON logging enabled'" C-m
tmux send-keys -t lunabot:10 "echo '═══════════════════════════════════════════'" C-m
tmux send-keys -t lunabot:10 "ros2 run lunabot_perception enhanced_hazard_detector" C-m

# Switch to control window
tmux select-window -t lunabot:0

echo ""
echo "════════════════════════════════════════════════════════════════════════════"
echo "✅ ALL 10 COMPONENTS LAUNCHED IN TMUX"
echo "════════════════════════════════════════════════════════════════════════════"
echo ""
echo "📋 TMUX WINDOWS:"
echo "   [0] control       - Control panel (this window)"
echo "   [1] env_monitor   - Environment Monitor"
echo "   [2] scan_relay    - Scan Relay"
echo "   [3] slam          - SLAM Toolbox"
echo "   [4] slam_odom     - SLAM Odometry Bridge"
echo "   [5] static_tf     - Static TF Publisher (map->odom)"
echo "   [6] nav2          - Nav2 Navigation Stack"
echo "   [7] rviz          - RViz2 Visualization"
echo "   [8] frontier      - Frontier Explorer"
echo "   [9] path_tracker  - Path Tracker"
echo "   [10] hazard       - Enhanced Hazard Detection"
echo ""
echo "🎮 TMUX COMMANDS:"
echo "   Attach to session:  tmux attach -t lunabot"
echo "   Switch windows:     Ctrl+b then 0-10"
echo "   Detach:             Ctrl+b then d"
echo "   Kill session:       tmux kill-session -t lunabot"
echo ""
echo "📁 OUTPUT LOCATIONS:"
echo "   • Hazard Images: ~/lunabot_hazards/*.jpg"
echo "   • Detection Log: ~/lunabot_hazards/hazard_detections.json"
echo "   • Path Logs: ~/lunabot_path_logs/*.csv"
echo ""
echo "🔍 MONITORING COMMANDS:"
echo "   ros2 topic echo /hazard_alerts         # Watch detections"
echo "   ros2 topic echo /path_footprint        # Watch path coordinates"
echo "   ls -lht ~/lunabot_hazards/             # Check images"
echo "   tail -f ~/lunabot_path_logs/path_*.csv # Watch coordinates"
echo "   ./VIEW_HAZARD_LOG.sh                   # View JSON log"
echo "   ./EXPORT_HAZARD_DATA.sh                # Export data"
echo ""
echo "════════════════════════════════════════════════════════════════════════════"
echo "🎯 ATTACHING TO TMUX SESSION..."
echo "════════════════════════════════════════════════════════════════════════════"

sleep 2
tmux attach -t lunabot
