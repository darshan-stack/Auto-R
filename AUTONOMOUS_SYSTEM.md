# ISRO LunaBot Challenge - Autonomous Navigation System
## Problem Statement #25169: Autonomous Navigation for Lunar Habitats

## 🚀 Quick Start (Industry-Ready System)

### 1. Start Isaac Sim
```bash
# Open Isaac Sim 4.5
# Load scene with nova_carter_ROS robot
# Press PLAY button
```

### 2. Launch Complete Autonomous System
```bash
cd /home/aids/lunabot_ws
chmod +x start_autonomous.sh
./start_autonomous.sh
```

**This single command launches everything:**
- ✅ Environmental monitoring (temperature, O2, pressure)
- ✅ 3D LiDAR to 2D laser scan converter
- ✅ Static TF publisher (map ↔ odom)
- ✅ Nav2 navigation stack (AMCL + Controller + Planner + Behaviors)
- ✅ RViz2 visualization
- ✅ **Autonomous waypoint patrol (robot moves automatically!)**

---

## 📊 System Architecture

```
Isaac Sim (nova_carter_ROS)
    │
    ├──> /chassis/odom ──────────────> AMCL Localization
    ├──> /front_3d_lidar/lidar_points ──> PointCloud→LaserScan ──> /scan ──> Costmaps
    └──> /cmd_vel ←───────────────────── DWB Controller
         │
         └──> Robot Motion (Autonomous)

Environmental Monitor
    ├──> /habitat/temperature
    ├──> /habitat/oxygen_level
    ├──> /habitat/pressure
    └──> /habitat/alerts

Autonomous Patrol
    └──> Sends NavigateToPose goals automatically
         Waypoints: (2,0) → (2,2) → (0,2) → (0,0) → Loop
```

---

## 🎯 What the Robot Does (Autonomous Mode)

Once launched, the robot **automatically**:

1. **Waits 15 seconds** for navigation stack to initialize
2. **Starts patrol mission** with 4 predefined waypoints
3. **Navigates autonomously** to each waypoint
4. **Avoids obstacles** using LiDAR costmaps
5. **Loops continuously** after completing all waypoints
6. **Monitors environment** (temperature, O2, pressure)
7. **Publishes alerts** when parameters go out of range

### Patrol Waypoints
- **Waypoint 1:** (2.0m, 0.0m) - Move forward
- **Waypoint 2:** (2.0m, 2.0m) - Turn left and move
- **Waypoint 3:** (0.0m, 2.0m) - Turn left and move
- **Waypoint 4:** (0.0m, 0.0m) - Return to start

---

## 🖥️ Monitoring the System

### Terminal Output
Watch the autonomous patrol node:
```bash
ros2 topic echo /patrol/status
```

### Environmental Monitoring
```bash
# Temperature
ros2 topic echo /habitat/temperature

# Oxygen level
ros2 topic echo /habitat/oxygen_level

# Pressure
ros2 topic echo /habitat/pressure

# Alerts
ros2 topic echo /habitat/alerts

# System status
ros2 topic echo /habitat/system_status
```

### Navigation Status
```bash
# Current robot velocity
ros2 topic echo /cmd_vel

# Robot position
ros2 topic echo /chassis/odom

# Laser scan data
ros2 topic echo /scan

# Navigation goal status
ros2 topic echo /navigate_to_pose/_action/status
```

### RViz Visualization
RViz automatically opens showing:
- 🗺️ Map
- 📍 Robot position
- 🔴 Laser scans
- 🟦 Local costmap (blue/purple)
- 🟩 Global costmap (green)
- 🎯 Current navigation goal
- 📏 Planned path (green line)
- 🔵 DWB trajectory predictions

---

## 🛠️ Manual Control (If Needed)

### Send Custom Navigation Goal
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 3.0, y: 1.5, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}"
```

### Stop Autonomous Patrol
```bash
# Press Ctrl+C in the launch terminal
# Or kill the patrol node:
ros2 node kill /autonomous_patrol
```

### Check TF Transforms
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map base_link
```

---

## 📋 ISRO Challenge Features Checklist

✅ **Autonomous Navigation**
   - Nav2 stack with AMCL localization
   - DWB local planner with 7 trajectory critics
   - NavFn global planner
   - Automatic waypoint following

✅ **Mapping**
   - Pre-built map (400x400 pixels, 20m x 20m)
   - Real-time costmap updates from LiDAR
   - Static + obstacle layers + inflation

✅ **Obstacle Detection & Avoidance**
   - 3D LiDAR point cloud processing
   - 2D laser scan generation
   - Dynamic obstacle detection
   - Trajectory cost evaluation (BaseObstacle critic)

✅ **Environmental Monitoring**
   - Temperature monitoring (ideal: 20-24°C)
   - Oxygen level monitoring (ideal: 19.5-23.5%)
   - Pressure monitoring (ideal: 99-103 kPa)
   - Automatic alert system
   - Robot activity detection

✅ **Routine Patrol**
   - Continuous autonomous waypoint navigation
   - Automatic loop after completion
   - Progress feedback
   - Error recovery

✅ **Alert & Signaling**
   - Out-of-range parameter warnings
   - Navigation failure alerts
   - Status publishing
   - Real-time monitoring

---

## 🐛 Troubleshooting

### Robot Not Moving

**Problem:** Robot stays in place after launch

**Solutions:**
1. Check Isaac Sim is running and Play button is pressed
2. Verify topics are publishing:
   ```bash
   ros2 topic hz /chassis/odom
   ros2 topic hz /front_3d_lidar/lidar_points
   ```
3. Check navigation server:
   ```bash
   ros2 action list
   # Should show /navigate_to_pose
   ```
4. Verify TF frames:
   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   ```

### RViz Not Opening

**Problem:** RViz2 doesn't launch

**Solutions:**
1. Launch with `use_rviz:=false` then manually:
   ```bash
   rviz2 -d ~/lunabot_ws/src/lunabot_navigation/rviz/navigation.rviz
   ```
2. Check display server:
   ```bash
   echo $DISPLAY
   ```

### Navigation Fails Immediately

**Problem:** Robot accepts goal but fails instantly

**Solutions:**
1. Check laser scan data:
   ```bash
   ros2 topic echo /scan --once
   ```
2. Verify costmaps in RViz (should see blue/green areas)
3. Check for TF transform errors in terminal
4. Reduce waypoint distance (edit autonomous_patrol.py)

### LiDAR Not Working

**Problem:** No /scan topic

**Solutions:**
1. Check point cloud input:
   ```bash
   ros2 topic hz /front_3d_lidar/lidar_points
   ```
2. Restart pointcloud_to_scan converter:
   ```bash
   ros2 run lunabot_navigation pointcloud_to_scan
   ```
3. Check frame names match (front_3d_lidar)

### Environmental Monitor Not Publishing

**Problem:** No habitat topics

**Solutions:**
1. Check if node is running:
   ```bash
   ros2 node list | grep environment
   ```
2. Restart manually:
   ```bash
   ros2 run lunabot_maintenance env_monitor
   ```

---

## 🎬 Recording Demo Video

For ISRO submission, record:

1. **System Startup** - Show all nodes launching
2. **RViz View** - Display map, robot, costmaps, path planning
3. **Isaac Sim View** - Show robot moving in simulation
4. **Terminal Monitoring** - Environmental data, patrol status
5. **Complete Loop** - At least one full 4-waypoint patrol
6. **Obstacle Avoidance** - (Optional) Place obstacle and show avoidance

### Recording Command
```bash
# Record all topics for later playback
ros2 bag record -a -o isro_demo
```

---

## 🔧 Advanced Configuration

### Adjust Patrol Waypoints
Edit: `src/lunabot_navigation/lunabot_navigation/autonomous_patrol.py`
```python
self.waypoints = [
    (x1, y1, yaw1),
    (x2, y2, yaw2),
    # Add more waypoints...
]
```

### Tune Navigation Parameters
Edit: `src/lunabot_navigation/config/nav2_params.yaml`
- Increase max velocity: `max_vel_x: 0.8`
- Adjust footprint size
- Modify critic weights

### Change Environmental Thresholds
Edit: `src/lunabot_maintenance/lunabot_maintenance/env_monitor.py`
```python
# Temperature range
if temp < 18 or temp > 26:
    # Adjust thresholds here
```

---

## 📦 Package Information

- **lunabot_navigation:** Navigation, path planning, autonomous patrol
- **lunabot_maintenance:** Environmental monitoring, alerts
- **ROS 2 Humble** on Ubuntu 22.04
- **Isaac Sim 4.5** for simulation
- **Nav2** for navigation stack

---

## 🏆 Competition Readiness

This system demonstrates:
- ✅ Industry-standard architecture
- ✅ Robust error handling
- ✅ Continuous operation
- ✅ Real-time monitoring
- ✅ Autonomous decision making
- ✅ Sensor fusion (LiDAR + Odom)
- ✅ Production-quality code
- ✅ Complete documentation

**Ready for ISRO submission!** 🚀🌙
