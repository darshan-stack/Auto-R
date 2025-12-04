#!/usr/bin/env python3
"""
SLAM Odometry Bridge - Uses SLAM pose as odometry for navigation
This allows navigation to work even when Isaac Sim odometry isn't available
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class SlamOdometryBridge(Node):
    def __init__(self):
        super().__init__('slam_odometry_bridge')
        
        # Subscribe to SLAM pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )
        
        # Publish odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        self.last_pose = None
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('🔄 SLAM Odometry Bridge Started')
        self.get_logger().info('   Subscribing to: /pose (SLAM pose)')
        self.get_logger().info('   Publishing to: /odom (for navigation)')
        
    def pose_callback(self, msg):
        # Create odometry message from SLAM pose
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Copy pose
        odom.pose = msg.pose
        
        # Calculate velocity if we have previous pose
        current_time = self.get_clock().now()
        if self.last_pose is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt > 0:
                dx = msg.pose.pose.position.x - self.last_pose.pose.pose.position.x
                dy = msg.pose.pose.position.y - self.last_pose.pose.pose.position.y
                
                vx = dx / dt
                vy = dy / dt
                
                # Calculate angular velocity
                # For simplicity, set to 0 (can be improved)
                vth = 0.0
                
                odom.twist.twist.linear.x = vx
                odom.twist.twist.linear.y = vy
                odom.twist.twist.angular.z = vth
        
        # Publish
        self.odom_pub.publish(odom)
        
        # Save for next iteration
        self.last_pose = msg
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    bridge = SlamOdometryBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info('🛑 SLAM odometry bridge stopped')
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
