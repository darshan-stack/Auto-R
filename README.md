# Auto-R: Autonomous Lunar Rover Navigation System

🤖 **Full autonomous exploration and navigation system for lunar rovers using ROS 2 Humble, Nav2, and Isaac Sim**

## 🌟 Features

- ✅ **Frontier-Based Exploration** - Autonomous mapping of unknown environments
- ✅ **SLAM with Map Building** - Real-time mapping using SLAM Toolbox
- ✅ **Nav2 Navigation Stack** - Professional path planning and obstacle avoidance
- ✅ **Hazard Detection System** - Real-time obstacle and anomaly detection with image capture
- ✅ **Environmental Monitoring** - Real-time telemetry and alerts
- ✅ **Isaac Sim Integration** - Tested with NVIDIA Isaac Sim nova_carter_ROS
- ✅ **RViz2 Visualization** - Live visualization of maps, paths, and robot state
- ✅ **Control Panel Alerts** - Automatic hazard image transmission with GPS location

## 📦 Packages

- **lunabot_navigation** - Navigation, exploration, and SLAM configuration
- **lunabot_maintenance** - Environmental monitoring, scan relay, odometry bridge
- **lunabot_perception** - Sensor processing
- **lunar_bot_pkg** - Core robot package

## 🚀 Quick Start

### Prerequisites
- ROS 2 Humble
- Nav2
- SLAM Toolbox
- Isaac Sim (optional, for simulation)

### Installation

```bash
cd ~/lunabot_ws
colcon build --symlink-install
source install/setup.bash
```

### Running the System

**After pressing Play in Isaac Sim**, run these commands in separate terminals:

#### Terminal 1: Environmental Monitor
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run lunabot_maintenance env_monitor
```

#### Terminal 2: Scan Relay
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run lunabot_maintenance scan_relay
```

#### Terminal 3: SLAM Toolbox
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=src/lunabot_navigation/config/slam_params.yaml \
  use_sim_time:=true
```

#### Terminal 4: SLAM Odometry Bridge
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run lunabot_maintenance slam_odom_bridge
```

#### Terminal 5: Static TF Publisher
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --frame-id map --child-frame-id odom \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0
```

#### Terminal 6: Nav2 Stack
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  params_file:=src/lunabot_navigation/config/nav2_params.yaml \
  map:=src/lunabot_navigation/maps/luna_map.yaml
```

#### Terminal 7: RViz2
```bash
unset GTK_PATH && unset LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run rviz2 rviz2 -d src/lunabot_navigation/rviz/navigation.rviz
```

#### Terminal 8: Frontier Explorer
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run lunabot_navigation frontier_explorer
```

#### Terminal 9: Hazard Detector (NEW)
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run lunabot_perception hazard_detector
```

#### Terminal 10: Control Panel (NEW)
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run lunabot_perception control_panel
```

**Or use the provided script:**
```bash
./START_EXPLORATION.sh
```

## 🗺️ Exploration Modes

### 1. Frontier-Based Exploration (Recommended)
Automatically explores the entire environment without repeating paths:
```bash
ros2 run lunabot_navigation frontier_explorer
```

### 2. Fixed Waypoint Patrol
Patrols predefined waypoints in a loop:
```bash
ros2 run lunabot_navigation autonomous_patrol
```

## 🔧 Configuration Files

- `src/lunabot_navigation/config/nav2_params.yaml` - Nav2 navigation parameters
- `src/lunabot_navigation/config/slam_params.yaml` - SLAM Toolbox configuration
- `src/lunabot_navigation/rviz/navigation.rviz` - RViz visualization config
- `src/lunabot_navigation/maps/luna_map.yaml` - Initial map configuration

## 📊 Key Nodes

### Navigation
- **frontier_explorer** - Frontier-based autonomous exploration
- **autonomous_patrol** - Fixed waypoint patrol system

### Maintenance
- **env_monitor** - Environmental telemetry and alerts
- **scan_relay** - LiDAR scan topic relay
- **slam_odom_bridge** - Converts SLAM pose to odometry

### Perception (NEW)
- **hazard_detector** - Real-time obstacle and anomaly detection
- **control_panel** - Hazard alert receiver and logger

## 🛠️ Diagnostic Commands

```bash
# Check all topics
ros2 topic list

# Monitor map updates
ros2 topic hz /map

# Check SLAM pose
ros2 topic echo /slam_toolbox/pose

# View odometry
ros2 topic echo /odom

# Check scan data
ros2 topic hz /scan

# Save current map
ros2 run nav2_map_server map_saver_cli -f ~/habitat_map
```

## 🏗️ Building

```bash
# Build all packages
colcon build --symlink-install

# Build specific package
colcon build --packages-select lunabot_navigation --symlink-install

# Clean build
rm -rf build install log
colcon build --symlink-install
```

## 📝 System Architecture

```
Isaac Sim (nova_carter_ROS)
    ↓
Scan Relay (/back_2d_lidar/scan → /scan)
    ↓
SLAM Toolbox (Mapping)
    ↓
SLAM Odom Bridge (/slam_pose → /odom)
    ↓
Nav2 Stack (Navigation)
    ↓
Frontier Explorer (Autonomous Goals)
    ↓
Robot Motion
```

## 🎯 Use Cases

- Lunar habitat exploration and mapping
- Autonomous rover navigation
- Unknown environment exploration
- SLAM-based navigation research
- Nav2 testing and development

## 📄 License

MIT License

## 👥 Contributors

Developed for ISRO LunaBot Challenge

## 🔗 Links

- [ROS 2 Humble](https://docs.ros.org/en/humble/)
- [Nav2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Isaac Sim](https://developer.nvidia.com/isaac-sim)
