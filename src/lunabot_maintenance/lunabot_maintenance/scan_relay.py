#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        
        # Subscribe to Isaac Sim's back 2D lidar
        self.subscription = self.create_subscription(
            LaserScan,
            '/back_2d_lidar/scan',
            self.scan_callback,
            10
        )
        
        # Publish to /scan for SLAM
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        
        self.get_logger().info('🔄 Scan Relay Started: /back_2d_lidar/scan → /scan')
        self.message_count = 0
        
    def scan_callback(self, msg):
        # Simply republish the message to /scan
        self.publisher.publish(msg)
        
        self.message_count += 1
        if self.message_count % 50 == 0:  # Log every 50 messages
            self.get_logger().info(f'✓ Relayed {self.message_count} scans ({msg.ranges.__len__()} points per scan)')

def main(args=None):
    rclpy.init(args=args)
    relay = ScanRelay()
    
    try:
        rclpy.spin(relay)
    except KeyboardInterrupt:
        relay.get_logger().info('🛑 Scan relay stopped')
    finally:
        relay.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
