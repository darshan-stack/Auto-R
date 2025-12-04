#!/usr/bin/env python3
"""
Control Panel Alert Receiver
Receives hazard alerts and displays them for operators
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from datetime import datetime


class ControlPanelReceiver(Node):
    def __init__(self):
        super().__init__('control_panel_receiver')
        
        # Subscribe to hazard alerts
        self.alert_sub = self.create_subscription(
            String, '/hazard_alerts', 
            self.alert_callback, 10)
        
        # Alert log file
        self.log_dir = os.path.expanduser('~/lunabot_control_panel')
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir, 'hazard_log.txt')
        
        self.get_logger().info('📡 Control Panel Receiver Started')
        self.get_logger().info(f'📋 Logging alerts to: {self.log_file}')
        
    def alert_callback(self, msg):
        """Receive and display hazard alerts"""
        try:
            alert = json.loads(msg.data)
            
            # Display alert
            self.get_logger().warn('=' * 80)
            self.get_logger().warn('🚨 HAZARD ALERT RECEIVED 🚨')
            self.get_logger().warn('=' * 80)
            self.get_logger().warn(f"Detection ID: {alert.get('detection_id', 'N/A')}")
            self.get_logger().warn(f"Timestamp: {alert.get('timestamp', 'N/A')}")
            self.get_logger().warn(f"Hazard Type: {alert.get('hazard_type', 'Unknown').upper()}")
            self.get_logger().warn(f"Distance: {alert.get('distance', 0):.2f} meters")
            
            if 'location' in alert:
                loc = alert['location']
                self.get_logger().warn(f"Location: X={loc['x']:.2f}m, Y={loc['y']:.2f}m, Z={loc['z']:.2f}m")
            
            if 'anomaly_scores' in alert:
                scores = alert['anomaly_scores']
                self.get_logger().warn(f"Anomaly Scores: Color={scores['color']:.2%}, Edge={scores['edge_density']:.2%}")
            
            self.get_logger().warn(f"📷 Image: {alert.get('image_path', 'N/A')}")
            self.get_logger().warn('=' * 80)
            
            # Log to file
            self.log_alert(alert)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse alert: {e}')
    
    def log_alert(self, alert):
        """Log alert to file"""
        with open(self.log_file, 'a') as f:
            f.write('\n' + '='*80 + '\n')
            f.write(f"HAZARD ALERT - {alert.get('timestamp', 'N/A')}\n")
            f.write(json.dumps(alert, indent=2))
            f.write('\n' + '='*80 + '\n')


def main(args=None):
    rclpy.init(args=args)
    receiver = ControlPanelReceiver()
    
    try:
        rclpy.spin(receiver)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
