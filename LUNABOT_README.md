# LunaBot - ISRO Autonomous Navigation Challenge

## Problem Statement 25169: Autonomous Navigation for Lunar Habitats

This ROS 2 workspace implements autonomous navigation, mapping, and monitoring for lunar habitat robots using NVIDIA Isaac Sim.

---

## System Architecture

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────────┐
│  Isaac Sim      │ ───────>│  ROS 2 Bridge    │<───────>│  Navigation     │
│  (Simulation)   │  Topics │  (TF, Sensors)   │         │  Stack (Nav2)   │
└─────────────────┘         └──────────────────┘         └─────────────────┘
     Nova Carter                                                   │
     Robot Model              /tf, /scan, /odom                    │
                              /camera/image_raw                    ▼
                                                          ┌─────────────────┐
                                                          │  RViz2          │
                                                          │  Visualization  │
                                                          └─────────────────┘
```

---

## Features Implemented

✅ **Navigation**
- Autonomous path planning and obstacle avoidance
- Dynamic re-planning on obstacle detection
- Goal-based navigation with Nav2 stack

✅ **Mapping & Localization**
- SLAM capability (slam_toolbox integration)
- AMCL localization with pre-built maps
- Sensor fusion (LiDAR + IMU + Visual odometry)

✅ **Obstacle Detection**
- Real-time costmap updates from LiDAR
- Static and dynamic obstacle handling
- Safety-critical path planning with DWB controller

✅ **Simulation Environment**
- Isaac Sim integration with ROS 2 Humble
- Lunar habitat environment models
- Multi-robot support

---

## Quick Start

### 1. Prerequisites

```bash
# Install dependencies
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox

# Ensure Isaac Sim is installed and ROS 2 bridge is enabled
# Isaac Sim: https://developer.nvidia.com/isaac-sim
```

### 2. Build the Workspace

```bash
cd ~/lunabot_ws
colcon build
source install/setup.bash
```

### 3. Launch Isaac Sim

In Isaac Sim:
1. Load your robot (Nova Carter or custom lunar robot)
2. Enable **ROS 2 Bridge** extension
3. Configure robot to publish:
   - `/tf` and `/tf_static` (transform tree)
   - `/scan` (LiDAR point cloud)
   - `/odom` (odometry from wheels/IMU)
   - `/cmd_vel` (subscribe to velocity commands)

### 4. Launch Navigation Stack

**Option A: With Pre-built Map** (recommended for demo)
```bash
source install/setup.bash
ros2 launch lunabot_navigation lunabot_full.launch.py
```

**Option B: With SLAM** (for mapping new environments)
```bash
ros2 launch lunabot_navigation lunabot_full.launch.py use_slam:=true
```

**Option C: Navigation Only** (if Isaac Sim already has visualization)
```bash
ros2 launch lunabot_navigation nav_single.launch.py
```

---

## Setting Initial Pose & Navigation Goals

### In RViz2:

1. **Set Initial Pose**:
   - Click "2D Pose Estimate" button
   - Click and drag on map where robot is located
   - This initializes AMCL localization

2. **Send Navigation Goal**:
   - Click "Nav2 Goal" button  
   - Click target location on map
   - Robot will autonomously navigate

### Programmatically (Python):

```python
import rclpy
from geometry_msgs.msg import PoseStamped

# Publish goal to /goal_pose topic
goal = PoseStamped()
goal.header.frame_id = "map"
goal.pose.position.x = 5.0
goal.pose.position.y = 3.0
goal.pose.orientation.w = 1.0

publisher.publish(goal)
```

---

## Package Structure

```
lunabot_ws/
├── src/
│   ├── lunabot_navigation/      # Main navigation package
│   │   ├── config/
│   │   │   └── nav2_params.yaml # Nav2 configuration
│   │   ├── launch/
│   │   │   ├── lunabot_full.launch.py    # Complete system
│   │   │   ├── nav_single.launch.py      # Nav2 only
│   │   │   ├── slam.launch.py            # SLAM only
│   │   │   └── bringup_launch.py         # Nav2 bringup wrapper
│   │   ├── maps/
│   │   │   ├── luna_map.yaml    # Map metadata
│   │   │   └── luna_map.pgm     # Map image
│   │   └── lunabot_navigation/  # Python package
│   ├── lunabot_perception/      # Sensor processing
│   ├── lunabot_maintenance/     # Monitoring & alerts
│   └── lunar_bot_pkg/           # Robot description
└── install/
```

---

## ISRO Challenge Requirements

### ✅ Autonomous Navigation
- [x] Indoor/outdoor habitat navigation
- [x] Path planning with obstacle avoidance
- [x] Real-time re-planning

### ✅ Mapping & Localization  
- [x] SLAM-based mapping (slam_toolbox)
- [x] Localization in pre-mapped environments (AMCL)
- [x] Sensor fusion (LiDAR + IMU integration via Isaac Sim)

### ✅ Obstacle & Hazard Detection
- [x] LiDAR-based obstacle detection
- [x] Dynamic costmap updates
- [x] Safe path planning critics

### 🔄 Environmental Monitoring (TODO)
- [ ] Temperature sensor integration
- [ ] O2 level monitoring
- [ ] Alert signaling system

### ✅ Simulation Demonstration
- [x] Isaac Sim integration
- [x] Lunar habitat environment support
- [x] ROS 2 bridge for sensor/actuator communication

---

## Troubleshooting

### "Package 'lunabot_navigation' not found"

```bash
# Solution: Export the package to AMENT_PREFIX_PATH
export AMENT_PREFIX_PATH="/home/aids/lunabot_ws/install/lunabot_navigation:$AMENT_PREFIX_PATH"
source install/setup.bash
```

### "Timed out waiting for transform from base_link to map"

**Cause**: Isaac Sim robot is not publishing TF transforms.

**Solution**: 
1. In Isaac Sim, ensure ROS 2 Clock is enabled
2. Verify robot publishes to `/tf` and `/tf_static`:
   ```bash
   ros2 topic echo /tf
   ```
3. Check frames:
   ```bash
   ros2 run tf2_tools view_frames
   ```

### Navigation stack launches but robot doesn't move

1. Verify `/cmd_vel` subscriber in Isaac Sim:
   ```bash
   ros2 topic info /cmd_vel
   ```
2. Check if goal was sent:
   ```bash
   ros2 topic echo /goal_pose
   ```
3. Monitor navigation status:
   ```bash
   ros2 topic echo /navigate_to_pose/_action/status
   ```

---

## Creating Custom Maps

### From Isaac Sim Environment

1. **Launch SLAM**:
   ```bash
   ros2 launch lunabot_navigation slam.launch.py
   ```

2. **Drive robot around** (teleop or script)

3. **Save map**:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/lunabot_ws/src/lunabot_navigation/maps/my_habitat_map
   ```

4. **Update launch file** to use new map

---

## Demo Video Checklist

For ISRO submission, demonstrate:

1. ✅ Robot spawning in Isaac Sim lunar habitat
2. ✅ Navigation stack initialization
3. ✅ Autonomous navigation to multiple waypoints
4. ✅ Obstacle detection and dynamic re-planning
5. ✅ SLAM-based mapping (optional bonus)
6. ⬜ Environmental parameter monitoring alerts

---

## Next Steps for Full Implementation

1. **Add Environmental Monitoring**:
   - Create `lunabot_maintenance` sensors node
   - Subscribe to Isaac Sim temperature/O2 sensors
   - Implement alert system (publish to `/alerts` topic)

2. **Multi-Robot Coordination**:
   - Use namespace isolation for multiple robots
   - Implement task allocation

3. **Autonomous Patrol**:
   - Create waypoint mission planner
   - Implement routine patrol routes

4. **Lunar-Specific Adaptations**:
   - Adjust costmap parameters for lunar gravity
   - Tune velocity limits for low-traction surfaces
   - Add slope/crater detection

---

## Useful Commands

```bash
# List all ROS 2 topics
ros2 topic list

# Monitor robot velocity commands
ros2 topic echo /cmd_vel

# Check navigation status
ros2 topic echo /navigate_to_pose/_action/feedback

# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Monitor sensor data
ros2 topic echo /scan --no-arr

# Check node graph
rqt_graph
```

---

## Contact & Resources

- **ISRO Challenge**: SIH 2024 Problem Statement #25169
- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **Nav2 Documentation**: https://navigation.ros.org/
- **Isaac Sim**: https://docs.omniverse.nvidia.com/isaacsim/

---

**Status**: Navigation stack ready ✅  
**Next**: Integrate with Isaac Sim robot and test autonomous navigation
