# LunaBot Navigation - Quick Command Reference

## Build Commands

```bash
# Build the navigation package
cd /home/aids/lunabot_ws
colcon build --packages-select lunabot_navigation

# Source the workspace
source install/setup.bash
export AMENT_PREFIX_PATH="/home/aids/lunabot_ws/install/lunabot_navigation:$AMENT_PREFIX_PATH"
```

## Launch Commands

### 1. Navigation Only (No RViz)
```bash
source install/setup.bash
export AMENT_PREFIX_PATH="/home/aids/lunabot_ws/install/lunabot_navigation:$AMENT_PREFIX_PATH"
ros2 launch lunabot_navigation nav_single.launch.py
```

### 2. Full System with RViz2 (RECOMMENDED)
```bash
source install/setup.bash
export AMENT_PREFIX_PATH="/home/aids/lunabot_ws/install/lunabot_navigation:$AMENT_PREFIX_PATH"
ros2 launch lunabot_navigation lunabot_full.launch.py
```

### 3. Full System without RViz
```bash
ros2 launch lunabot_navigation lunabot_full.launch.py use_rviz:=false
```

### 4. With SLAM (for mapping)
```bash
ros2 launch lunabot_navigation lunabot_full.launch.py use_slam:=true
```

### 5. RViz2 Only (if navigation already running)
```bash
rviz2 -d $(ros2 pkg prefix lunabot_navigation)/share/lunabot_navigation/rviz/navigation.rviz
```

## Send Navigation Goals

### Method 1: Using RViz2 (Easiest)
1. Launch full system with RViz
2. Click "2D Goal Pose" button in toolbar
3. Click on map and drag to set goal orientation
4. Robot will navigate autonomously

### Method 2: Command Line
```bash
# Navigate to position (2, 1) facing forward
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 2.0, y: 1.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}"

# Navigate to position (3, 3) facing 90° left
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 3.0, y: 3.0, z: 0.0},
      orientation: {z: 0.707, w: 0.707}
    }
  }
}"

# Navigate to position (-2, -2) facing backward (180°)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: -2.0, y: -2.0, z: 0.0},
      orientation: {z: 1.0, w: 0.0}
    }
  }
}"
```

### Method 3: Python Script
```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = rclpy.create_node('send_goal_node')
client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

goal_msg = NavigateToPose.Goal()
goal_msg.pose.header.frame_id = 'map'
goal_msg.pose.pose.position.x = 2.0
goal_msg.pose.pose.position.y = 1.0
goal_msg.pose.pose.orientation.w = 1.0

client.wait_for_server()
client.send_goal_async(goal_msg)
rclpy.spin(node)
```

## Set Initial Pose (for AMCL localization)

### In RViz2:
1. Click "2D Pose Estimate" button
2. Click on map where robot currently is
3. Drag to set orientation

### Command Line:
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}"
```

## Monitoring & Debugging

### Check Active Topics
```bash
ros2 topic list
```

### Monitor Robot Position
```bash
ros2 topic echo /odom
```

### Monitor Navigation Status
```bash
# Watch navigation goal status
ros2 action list
ros2 action info /navigate_to_pose

# Check costmaps
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /global_costmap/costmap --once
```

### Check TF Transforms
```bash
# Check transform tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

### View Scan Data
```bash
# Front LiDAR
ros2 topic echo /front_2d_lidar/scan

# Back LiDAR
ros2 topic echo /back_2d_lidar/scan
```

### Check Node Status
```bash
# List all running nodes
ros2 node list

# Get info about specific node
ros2 node info /controller_server
ros2 node info /planner_server
ros2 node info /bt_navigator
```

## Complete Workflow

### Step 1: Start Isaac Sim
1. Open Isaac Sim
2. Load your robot scene (Nova Carter)
3. Enable ROS2 bridge
4. Press PLAY button

### Step 2: Verify Isaac Sim Topics
```bash
# Check that Isaac Sim is publishing
ros2 topic list | grep -E "(scan|odom|tf)"
```

### Step 3: Launch Navigation
```bash
cd /home/aids/lunabot_ws
source install/setup.bash
export AMENT_PREFIX_PATH="/home/aids/lunabot_ws/install/lunabot_navigation:$AMENT_PREFIX_PATH"
ros2 launch lunabot_navigation lunabot_full.launch.py
```

### Step 4: Set Initial Pose in RViz
1. Use "2D Pose Estimate" tool
2. Click where robot is on map

### Step 5: Send Navigation Goal
1. Use "2D Goal Pose" tool in RViz
2. Click destination on map
3. Watch robot navigate autonomously!

## Troubleshooting

### Navigation stack not starting
```bash
# Check if all topics are available
ros2 topic list

# Verify TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

### Robot not moving
```bash
# Check if cmd_vel is being published
ros2 topic echo /cmd_vel

# Check navigation node status
ros2 lifecycle list /controller_server
ros2 lifecycle get /controller_server
```

### Costmap errors
```bash
# Check if map is loaded
ros2 topic echo /map --once

# Verify scan data
ros2 topic hz /front_2d_lidar/scan
```

### AMENT_PREFIX_PATH issues
```bash
# Always export before launching
export AMENT_PREFIX_PATH="/home/aids/lunabot_ws/install/lunabot_navigation:$AMENT_PREFIX_PATH"

# Or add to ~/.bashrc for permanent fix
echo 'export AMENT_PREFIX_PATH="/home/aids/lunabot_ws/install/lunabot_navigation:$AMENT_PREFIX_PATH"' >> ~/.bashrc
```

## Performance Tuning

### Increase planner frequency
Edit `config/nav2_params.yaml`:
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 30.0  # Default: 20.0
```

### Adjust costmap update rate
```yaml
local_costmap:
  ros__parameters:
    update_frequency: 10.0  # Default: 5.0
```

### Change robot velocity limits
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.5  # Increase for faster movement
      max_vel_theta: 2.0
```

## ISRO Challenge Demo

### Record Navigation Demo
```bash
# Start recording
ros2 bag record -a -o lunabot_demo

# Run navigation demo
# ... perform autonomous navigation ...

# Stop recording (Ctrl+C)

# Play back
ros2 bag play lunabot_demo
```

### Create Demo Video
```bash
# Install screen recorder
sudo apt install simplescreenrecorder

# Record:
# 1. Start navigation with RViz
# 2. Start screen recorder
# 3. Send multiple navigation goals
# 4. Show obstacle avoidance
# 5. Stop recording
```
