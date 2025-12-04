#!/usr/bin/env python3
"""
Topic Bridge for Isaac Sim to Nav2
Remaps Isaac Sim topics to Nav2-compatible topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import math

class IsaacSimBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')
        
        # Subscribe to Isaac Sim topics
        self.odom_sub = self.create_subscription(
            Odometry,
            '/chassis/odom',
            self.odom_callback,
            10)
        
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/front_3d_lidar/lidar_points',
            self.lidar_callback,
            10)
        
        # Publish to Nav2 topics
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        self.get_logger().info('Isaac Sim Bridge started')
        self.get_logger().info('Remapping: /chassis/odom -> /odom')
        self.get_logger().info('Converting: /front_3d_lidar/lidar_points -> /scan')
    
    def odom_callback(self, msg):
        # Remap chassis to base_link
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        self.odom_pub.publish(msg)
    
    def lidar_callback(self, msg):
        # Convert 3D point cloud to 2D laser scan
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = 'base_link'
        
        # Scan parameters
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0  # 1 degree
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        
        # Initialize ranges
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [float('inf')] * num_readings
        scan.intensities = [0.0] * num_readings
        
        # Convert point cloud to 2D scan (only points near ground plane)
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            
            # Only use points near the robot's height (within ±0.5m of z=0)
            if abs(z) < 0.5:
                range_val = math.sqrt(x*x + y*y)
                if scan.range_min < range_val < scan.range_max:
                    angle = math.atan2(y, x)
                    index = int((angle - scan.angle_min) / scan.angle_increment)
                    if 0 <= index < num_readings:
                        if range_val < scan.ranges[index]:
                            scan.ranges[index] = range_val
        
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    bridge = IsaacSimBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
