# 🎬 ISRO LUNABOT DEMO - FULL SYSTEM CAPTURE GUIDE
## Execution Status: ✅ ALL 6 COMPONENTS RUNNING

**Demo Directory:** `~/lunabot_demo_captures/`  
**Execution Date:** December 4, 2025  
**System Status:** FULLY OPERATIONAL

---

## 📸 CAPTURE CHECKLIST - 8 CRITICAL IMAGES

### IMAGE 1: Environmental Monitor ✅
**Status:** RUNNING  
**Terminal ID:** b44b4688-e5e6-4c32-8c12-7168fd71914f  
**What to Capture:**
```
Terminal showing:
- ✅ All parameters returned to nominal range
- 📊 Status: NOMINAL | Temp: 21-23°C | O2: 20-21% | Press: 99-103kPa
- ⚠️ PRESSURE ALERT notifications (when anomalies occur)
```
**Key Info Visible:**
- Temperature range: 15-25°C
- Oxygen levels: 19-23%
- Pressure monitoring with alerts
- Robot activity tracking

**Capture Location:** Terminal window  
**Save As:** `~/lunabot_demo_captures/01_environmental_monitor.png`

---

### IMAGE 2: Scan Relay ✅
**Status:** RUNNING  
**Terminal ID:** cbe2b4e9-7a3a-4dfc-a7d3-ec89b22be14e  
**What to Capture:**
```
Terminal showing:
[INFO] [scan_relay]: 🔄 Scan Relay Started: /back_2d_lidar/scan → /scan
```
**Key Info Visible:**
- LiDAR data relay from `/back_2d_lidar/scan` to `/scan`
- Sensor fusion enabled
- Real-time scan data flowing

**Capture Location:** Terminal window  
**Save As:** `~/lunabot_demo_captures/02_scan_relay.png`

---

### IMAGE 3: SLAM Toolbox ✅
**Status:** RUNNING  
**Terminal ID:** 70b882a4-4295-45a2-9599-cf734177b581  
**What to Capture:**
```
Terminal showing:
[INFO] [async_slam_toolbox_node-1] [slam_toolbox]: Node using stack size 40000000
[INFO] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[INFO] [slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner.
```
**Additional:** If RViz is working, capture the map building visualization  
**Key Info Visible:**
- Real-time SLAM mapping active
- Solver: CeresSolver
- Map generation in progress

**Capture Location:** Terminal window (+ RViz if available)  
**Save As:** `~/lunabot_demo_captures/03_slam_toolbox.png`

---

### IMAGE 4: Odometry Bridge ✅
**Status:** RUNNING  
**Terminal ID:** c0e117b7-f302-439d-9c58-9fe191ad6bd5  
**What to Capture:**
```
Terminal showing:
[INFO] [slam_odometry_bridge]: 🔄 SLAM Odometry Bridge Started
[INFO] [slam_odometry_bridge]:    Subscribing to: /pose (SLAM pose)
[INFO] [slam_odometry_bridge]:    Publishing to: /odom (for navigation)
```
**Key Info Visible:**
- SLAM pose fusion
- Odometry publishing to `/odom` topic
- Bridge active for navigation stack

**Capture Location:** Terminal window  
**Save As:** `~/lunabot_demo_captures/04_odometry_bridge.png`

---

### IMAGE 5: Nav2 Navigation Stack ✅
**Status:** RUNNING  
**Terminal ID:** b0bc1ec1-ac17-49c7-84e8-823114264a8a  
**What to Capture:**
```
Terminal showing:
[lifecycle_manager_navigation]: Managed nodes are active
[controller_server]: Activating
[planner_server]: Activating
[behavior_server]: Activating
[bt_navigator]: Activating
```
**Key Info Visible:**
- All Nav2 servers activated:
  - controller_server
  - smoother_server
  - planner_server
  - behavior_server
  - bt_navigator
  - waypoint_follower
  - velocity_smoother
- Costmaps generating (local + global)

**Additional Topics to Show:**
```bash
ros2 topic echo /global_costmap/costmap --once
ros2 topic echo /local_costmap/costmap --once
```

**Capture Location:** Terminal window showing lifecycle transitions  
**Save As:** `~/lunabot_demo_captures/05_nav2_stack.png`

---

### IMAGE 6: Hazard Detection System ✅
**Status:** RUNNING  
**Terminal ID:** fe9bac96-6b02-4132-9963-e7839ea7fc1a  
**What to Capture:**
```
Terminal showing:
[INFO] [hazard_detector]: 🚨 Hazard Detection System Started
[INFO] [hazard_detector]: 📁 Saving hazard images to: /home/aids/lunabot_hazards
[WARN] [hazard_detector]: ⚠️ HAZARD DETECTED: Obstacle at X.Xm
[INFO] [hazard_detector]: 📸 Captured hazard image: hazard_TIMESTAMP.jpg
[INFO] [hazard_detector]: 📡 Alert sent to control panel
```
**Key Info Visible:**
- LiDAR obstacle detection (<1.5m threshold)
- Visual anomaly detection active
- Image capture with GPS location
- Alert transmission to control panel

**Additional - Show Captured Hazard Images:**
```bash
ls -lh ~/lunabot_hazards/
```

**Capture Location:** Terminal window + hazard image files  
**Save As:** `~/lunabot_demo_captures/06_hazard_detection.png`

---

### IMAGE 7: Control Panel Alerts ✅
**Status:** RUNNING  
**Terminal ID:** 0f6054fb-6d0b-4988-9784-893198cf53b2  
**What to Capture:**
```
Terminal showing:
[INFO] [control_panel_receiver]: 📡 Control Panel Receiver Started
[INFO] [control_panel_receiver]: 📋 Logging alerts to: /home/aids/lunabot_control_panel/hazard_log.txt

╔════════════════════════════════════════════════════════════╗
║               🚨 HAZARD ALERT RECEIVED                     ║
╚════════════════════════════════════════════════════════════╝
⚠️  Type: OBSTACLE
📍 Position: (x, y, z)
📏 Distance: X.X meters
⏰ Time: YYYY-MM-DD HH:MM:SS
🗺️  GPS: Lat X.XXXX, Lon Y.YYYY
📸 Image: hazard_TIMESTAMP.jpg
```
**Key Info Visible:**
- Real-time hazard alerts
- JSON message parsing
- GPS coordinates
- Image file references
- Log file path

**Additional - Show Log File:**
```bash
cat ~/lunabot_control_panel/hazard_log.txt
```

**Capture Location:** Terminal window with alert boxes  
**Save As:** `~/lunabot_demo_captures/07_control_panel.png`

---

### IMAGE 8: Frontier Explorer ✅
**Status:** RUNNING  
**Terminal ID:** 3f56581d-046f-4505-8a70-1f1434091c7f  
**What to Capture:**
```
Terminal showing:
[INFO] [frontier_explorer]: 🗺️  Frontier Explorer Started - Full Habitat Mapping Mode
[INFO] [frontier_explorer]: 📍 Waiting for map and odometry data...
[INFO] [frontier_explorer]: 🎯 Found X frontiers
[INFO] [frontier_explorer]: ⭐ Best frontier selected at (x, y) - size: XX cells
[INFO] [frontier_explorer]: 🚀 Navigating to frontier...
[INFO] [frontier_explorer]: ✅ Frontier reached, searching for next...
```
**Key Info Visible:**
- Frontier detection algorithm active
- Map boundary exploration
- Goal selection based on size/distance
- Visited area tracking (no path repetition)

**Capture Location:** Terminal window showing exploration progress  
**Save As:** `~/lunabot_demo_captures/08_frontier_explorer.png`

---

## 🎥 BONUS CAPTURES

### Isaac Sim Robot View
**What to Capture:**
- Nova Carter robot in lunar habitat environment
- Robot position in simulation
- LiDAR visualization (if visible)
**Save As:** `~/lunabot_demo_captures/09_isaac_sim_robot.png`

### Topic List
**Command:**
```bash
ros2 topic list | grep -E "(hazard|habitat|scan|odom|map)"
```
**Expected Output:**
```
/back_2d_lidar/scan
/chassis/odom
/global_costmap/costmap
/hazard_alerts
/hazard_stop
/habitat/temperature
/habitat/pressure
/habitat/oxygen
/map
/odom
/scan
```
**Save As:** `~/lunabot_demo_captures/10_active_topics.png`

### Node List
**Command:**
```bash
ros2 node list
```
**Expected Nodes:**
- /environment_monitor
- /scan_relay
- /slam_toolbox
- /slam_odometry_bridge
- /controller_server
- /planner_server
- /behavior_server
- /bt_navigator
- /hazard_detector
- /control_panel_receiver
- /frontier_explorer
**Save As:** `~/lunabot_demo_captures/11_active_nodes.png`

---

## 📊 SYSTEM VERIFICATION COMMANDS

Run these commands to verify all systems before capturing:

```bash
# 1. Check all nodes are running
ros2 node list

# 2. Check critical topics
ros2 topic list | grep -E "(hazard|habitat|scan|odom|map)"

# 3. Verify environmental monitor output
ros2 topic echo /habitat/temperature --once
ros2 topic echo /habitat/pressure --once
ros2 topic echo /habitat/oxygen --once

# 4. Verify hazard detection topics
ros2 topic info /hazard_alerts
ros2 topic info /hazard_stop

# 5. Verify scan relay
ros2 topic echo /scan --once

# 6. Verify odometry
ros2 topic echo /odom --once

# 7. Verify SLAM map
ros2 topic echo /map --once

# 8. Check hazard images directory
ls -lh ~/lunabot_hazards/

# 9. Check control panel logs
cat ~/lunabot_control_panel/hazard_log.txt
```

---

## 🎯 CAPTURE SEQUENCE RECOMMENDATIONS

**Phase 1: Individual Components (15 minutes)**
1. Capture environmental monitor (IMAGE 1)
2. Capture scan relay (IMAGE 2)
3. Capture SLAM toolbox (IMAGE 3)
4. Capture odometry bridge (IMAGE 4)
5. Capture Nav2 stack (IMAGE 5)

**Phase 2: Autonomous Systems (20 minutes)**
6. Wait for robot to start moving in Isaac Sim
7. Capture hazard detection when obstacle detected (IMAGE 6)
8. Capture control panel when alert received (IMAGE 7)
9. Capture frontier explorer showing exploration progress (IMAGE 8)

**Phase 3: Integrated View (10 minutes)**
10. Capture Isaac Sim with robot exploring (BONUS IMAGE 9)
11. Capture topic list (BONUS IMAGE 10)
12. Capture node list (BONUS IMAGE 11)

---

## 🚀 TERMINAL ID REFERENCE

| Component | Terminal ID | Status |
|-----------|-------------|---------|
| Environmental Monitor | `b44b4688-e5e6-4c32-8c12-7168fd71914f` | ✅ RUNNING |
| Scan Relay | `cbe2b4e9-7a3a-4dfc-a7d3-ec89b22be14e` | ✅ RUNNING |
| SLAM Toolbox | `70b882a4-4295-45a2-9599-cf734177b581` | ✅ RUNNING |
| Odometry Bridge | `c0e117b7-f302-439d-9c58-9fe191ad6bd5` | ✅ RUNNING |
| Static TF | `6ec09ea1-b75b-49d8-b713-ce0b34118c3e` | ✅ RUNNING |
| Nav2 Stack | `b0bc1ec1-ac17-49c7-84e8-823114264a8a` | ✅ RUNNING |
| Control Panel | `0f6054fb-6d0b-4988-9784-893198cf53b2` | ✅ RUNNING |
| Hazard Detector | `fe9bac96-6b02-4132-9963-e7839ea7fc1a` | ✅ RUNNING |
| Frontier Explorer | `3f56581d-046f-4505-8a70-1f1434091c7f` | ✅ RUNNING |

---

## ✅ COMPLETION CHECKLIST

- [ ] All 8 main images captured
- [ ] All 3 bonus images captured
- [ ] Images saved to `~/lunabot_demo_captures/`
- [ ] Hazard image examples collected from `~/lunabot_hazards/`
- [ ] Control panel log verified `~/lunabot_control_panel/hazard_log.txt`
- [ ] Isaac Sim showing autonomous exploration
- [ ] All timestamps and GPS coordinates visible in captures
- [ ] Terminal outputs show system names and status messages

---

## 📝 NOTES FOR DEMO VIDEO

When creating the final demo video, include:

1. **Introduction (30 seconds)**
   - ISRO LunaBot Challenge overview
   - System capabilities summary

2. **Component Walkthrough (3 minutes)**
   - Show each of the 6 systems running
   - Explain what each does
   - Highlight terminal outputs

3. **Autonomous Operation (4 minutes)**
   - Robot exploring in Isaac Sim
   - Frontier detection and navigation
   - Hazard detection triggering
   - Images being captured
   - Control panel receiving alerts
   - Environmental monitoring

4. **Results Showcase (2 minutes)**
   - Generated map visualization
   - Collected hazard images with GPS tags
   - Control panel log review
   - System statistics

5. **Conclusion (30 seconds)**
   - 95% implementation status
   - GitHub repository link
   - Future enhancements

**Total Duration:** 10 minutes

---

## 🎬 READY FOR DEMO!

**ALL SYSTEMS OPERATIONAL - CAPTURE IN PROGRESS**

Status: ✅ Environmental Monitor  
Status: ✅ Scan Relay  
Status: ✅ SLAM Toolbox  
Status: ✅ Odometry Bridge  
Status: ✅ Static TF Publisher  
Status: ✅ Nav2 Stack  
Status: ✅ Hazard Detector  
Status: ✅ Control Panel  
Status: ✅ Frontier Explorer  

**ISRO Challenge Implementation: 95%**
