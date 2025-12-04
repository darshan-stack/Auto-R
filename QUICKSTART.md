# LunaBot ISRO Challenge - Quick Start Guide

## System Overview
Complete autonomous navigation system for lunar habitat exploration with environmental monitoring.

## Components
1. **Navigation Stack** - Nav2 with AMCL localization
2. **Environmental Monitoring** - Temperature, O2, Pressure sensors
3. **Obstacle Avoidance** - LiDAR-based detection
4. **Path Planning** - DWB local planner + NavFn global planner

## Quick Launch

### Method 1: Automated Demo Launch
```bash
cd /home/aids/lunabot_ws
./launch_demo.sh
```

### Method 2: Manual Launch

**Step 1: Start Isaac Sim**
- Open Isaac Sim
- Load the scene with nova_carter_ROS
- Press PLAY ▶️

**Step 2: Launch Environment Monitor**
```bash
cd /home/aids/lunabot_ws
source install/setup.bash
ros2 run lunabot_maintenance env_monitor
```

**Step 3: Launch Navigation (New Terminal)**
```bash
cd /home/aids/lunabot_ws
source install/setup.bash
export AMENT_PREFIX_PATH="/home/aids/lunabot_ws/install/lunabot_navigation:$AMENT_PREFIX_PATH"
ros2 launch lunabot_navigation nova_carter_nav.launch.py
```

**Step 4: Send Navigation Goal (New Terminal)**
```bash
# Simple goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Different positions
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 3.0, z: 0.0}, orientation: {z: 0.707, w: 0.707}}}}"
```

## Monitoring Commands

```bash
# Check all topics
ros2 topic list

# Monitor environmental status
ros2 topic echo /habitat/system_status

# Monitor temperature
ros2 topic echo /habitat/temperature

# Monitor O2 level
ros2 topic echo /habitat/oxygen_level

# Monitor pressure
ros2 topic echo /habitat/pressure

# Check robot odometry
ros2 topic echo /odom

# View navigation feedback
ros2 topic echo /navigate_to_pose/_action/feedback

# Check TF tree
ros2 run tf2_tools view_frames
```

## ISRO Challenge Features

### ✅ Implemented
1. **Autonomous Navigation** - Nav2 stack with path planning
2. **Mapping & Localization** - AMCL on pre-built map
3. **Obstacle Detection & Avoidance** - LiDAR-based costmaps
4. **Environmental Monitoring** - Temperature, O2, Pressure
5. **Alert System** - Automatic warnings for out-of-range parameters
6. **RViz Visualization** - Real-time robot state and planning

### Demo Scenarios

**Scenario 1: Routine Patrol**
```bash
# Send multiple waypoints
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
# Wait for completion, then:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: -2.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

**Scenario 2: Obstacle Avoidance**
- Place obstacles in Isaac Sim
- Send navigation goal
- Watch robot replan around obstacles in RViz

**Scenario 3: Environmental Monitoring**
- Watch terminal for environmental alerts
- System automatically detects and reports anomalies

## Troubleshooting

### Isaac Sim crashes
- Reduce LiDAR resolution in Isaac Sim
- Close other applications
- Disable extra robots (keep only nova_carter_ROS)

### Navigation not working
```bash
# Check if all topics are available
ros2 topic list | grep -E "(cmd_vel|odom|scan|tf)"

# Check TF tree
ros2 run tf2_tools tf2_echo map base_link

# Restart navigation
# Ctrl+C in navigation terminal, then relaunch
```

### No map visible in RViz
- Check map file exists: `ls ~/lunabot_ws/src/lunabot_navigation/maps/`
- Verify map topic: `ros2 topic echo /map --once`

## System Architecture

```
Isaac Sim (nova_carter_ROS)
    ↓ (publishes /cmd_vel, /odom, /scan, /tf)
    ↓
Static TF: map → odom
    ↓
Nav2 Stack
    ├─ Map Server (loads luna_map.yaml)
    ├─ AMCL (localization)
    ├─ Controller Server (DWB local planner)
    ├─ Planner Server (NavFn global planner)
    ├─ Behavior Server (recovery behaviors)
    └─ BT Navigator (behavior tree coordination)
    ↓
RViz2 (visualization)

Environment Monitor (parallel)
    ├─ Temperature sensor (simulated)
    ├─ O2 sensor (simulated)
    ├─ Pressure sensor (simulated)
    └─ Alert system
```

## Video Demo Recording Checklist

For ISRO submission, record:
- [ ] Robot starting from initial position
- [ ] Autonomous navigation to multiple waypoints
- [ ] Obstacle avoidance demonstration
- [ ] RViz showing costmaps and path planning
- [ ] Terminal showing environmental monitoring data
- [ ] Alert system triggering on parameter changes
- [ ] Robot completing full patrol circuit

## Contact & Support
- Package: lunabot_navigation, lunabot_maintenance
- ROS2: Humble
- Simulator: Isaac Sim 4.5
- GPU: RTX 4060 8GB
