#!/usr/bin/env python3
"""
PointCloud to LaserScan Converter for ISRO LunaBot
Converts 3D LiDAR point clouds to 2D laser scans for Nav2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
import math
import numpy as np

class PointCloudToScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_scan')
        
        # Parameters
        self.declare_parameter('min_height', -0.5)
        self.declare_parameter('max_height', 0.5)
        self.declare_parameter('angle_min', -3.14159)
        self.declare_parameter('angle_max', 3.14159)
        self.declare_parameter('angle_increment', 0.00872665)  # 0.5 degrees
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('scan_frame', 'front_3d_lidar')
        
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_frame = self.get_parameter('scan_frame').value
        
        # Subscriber to point cloud
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/front_3d_lidar/lidar_points',
            self.pointcloud_callback,
            10
        )
        
        # Publisher for laser scan
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        self.get_logger().info('✅ PointCloud to LaserScan converter started')
        self.get_logger().info(f'   Input: /front_3d_lidar/lidar_points')
        self.get_logger().info(f'   Output: /scan')
        self.get_logger().info(f'   Height filter: {self.min_height}m to {self.max_height}m')
        
    def pointcloud_callback(self, cloud_msg):
        """Convert PointCloud2 to LaserScan"""
        
        # Create LaserScan message
        scan = LaserScan()
        scan.header = cloud_msg.header
        scan.header.frame_id = self.scan_frame
        
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Calculate number of beams
        num_beams = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        scan.ranges = [float('inf')] * num_beams
        scan.intensities = [0.0] * num_beams
        
        # Read point cloud data
        try:
            points = point_cloud2.read_points(
                cloud_msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            )
            
            for point in points:
                x, y, z = point
                
                # Filter by height
                if z < self.min_height or z > self.max_height:
                    continue
                
                # Calculate range and angle
                range_val = math.sqrt(x * x + y * y)
                
                if range_val < self.range_min or range_val > self.range_max:
                    continue
                
                angle = math.atan2(y, x)
                
                # Calculate beam index
                if angle < self.angle_min or angle > self.angle_max:
                    continue
                
                beam_index = int((angle - self.angle_min) / self.angle_increment)
                
                if 0 <= beam_index < num_beams:
                    # Keep minimum range for each beam
                    if range_val < scan.ranges[beam_index]:
                        scan.ranges[beam_index] = range_val
                        
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')
            return
        
        # Publish scan
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToScan()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
