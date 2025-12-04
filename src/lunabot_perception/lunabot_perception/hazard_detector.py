#!/usr/bin/env python3
"""
Hazard Detection and Anomaly Detection System
Detects hazardous/anomalous objects during patrol and alerts control panel
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
from datetime import datetime


class HazardDetector(Node):
    def __init__(self):
        super().__init__('hazard_detector')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, '/front_stereo_camera/left/image_raw', 
            self.camera_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', 
            self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', 
            self.odom_callback, 10)
        
        # Publishers
        self.hazard_alert_pub = self.create_publisher(
            String, '/hazard_alerts', 10)
        self.cmd_vel_override_pub = self.create_publisher(
            Twist, '/hazard_stop', 10)
        
        # State variables
        self.current_pose = None
        self.latest_image = None
        self.hazard_detected = False
        self.detection_cooldown = 0
        
        # Detection parameters
        self.hazard_distance_threshold = 1.5  # meters
        self.anomaly_threshold = 0.7  # confidence threshold
        self.image_save_dir = os.path.expanduser('~/lunabot_hazards')
        
        # Create hazard image directory
        os.makedirs(self.image_save_dir, exist_ok=True)
        
        # Simple anomaly detection parameters
        self.baseline_intensity = None
        self.detection_count = 0
        
        self.get_logger().info('🚨 Hazard Detection System Started')
        self.get_logger().info(f'📁 Saving hazard images to: {self.image_save_dir}')
        
        # Timer for periodic checks
        self.timer = self.create_timer(0.5, self.detection_loop)
        
    def odom_callback(self, msg):
        """Update current robot position"""
        self.current_pose = msg.pose.pose
        
    def camera_callback(self, msg):
        """Process camera images"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Camera conversion error: {e}')
    
    def scan_callback(self, msg):
        """Process LiDAR scan for obstacle detection"""
        if self.detection_cooldown > 0:
            self.detection_cooldown -= 1
            return
        
        # Check for obstacles within danger zone
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[np.isnan(ranges)] = msg.range_max
        
        # Front cone detection (center ±30 degrees)
        front_cone_size = len(ranges) // 6  # ~30 degrees each side
        center_idx = len(ranges) // 2
        front_ranges = ranges[center_idx - front_cone_size:center_idx + front_cone_size]
        
        min_distance = np.min(front_ranges)
        
        if min_distance < self.hazard_distance_threshold:
            self.detect_hazard(min_distance, 'obstacle')
    
    def detect_anomaly_visual(self, image):
        """
        Simple visual anomaly detection using color/texture analysis
        Returns True if anomaly detected
        """
        if image is None:
            return False
        
        # Convert to HSV for better color analysis
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Detect unusual colors (e.g., bright colors, non-lunar surface)
        # Moon surface is typically gray/brown
        lower_unusual = np.array([0, 50, 50])    # Bright/saturated colors
        upper_unusual = np.array([180, 255, 255])
        
        mask = cv2.inRange(hsv, lower_unusual, upper_unusual)
        unusual_pixels = np.sum(mask > 0)
        total_pixels = image.shape[0] * image.shape[1]
        unusual_ratio = unusual_pixels / total_pixels
        
        # Also check for sharp edges (debris, equipment)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        edge_density = np.sum(edges > 0) / total_pixels
        
        # Detect if unusual
        is_anomaly = unusual_ratio > 0.1 or edge_density > 0.15
        
        return is_anomaly, unusual_ratio, edge_density
    
    def detect_hazard(self, distance, hazard_type):
        """Detect and report hazard"""
        if self.hazard_detected or not self.latest_image is not None:
            return
        
        # Perform visual anomaly detection
        is_anomaly, color_score, edge_score = self.detect_anomaly_visual(self.latest_image)
        
        if is_anomaly or hazard_type == 'obstacle':
            self.hazard_detected = True
            self.detection_count += 1
            
            # Capture image
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'hazard_{self.detection_count:04d}_{timestamp}.jpg'
            filepath = os.path.join(self.image_save_dir, filename)
            
            # Annotate image
            annotated_image = self.annotate_hazard_image(
                self.latest_image.copy(), distance, hazard_type, color_score, edge_score)
            
            cv2.imwrite(filepath, annotated_image)
            
            # Create alert message
            alert = self.create_alert_message(
                filepath, distance, hazard_type, color_score, edge_score)
            
            # Publish alert
            self.hazard_alert_pub.publish(String(data=json.dumps(alert)))
            
            # Log detection
            self.get_logger().warn(f'🚨 HAZARD DETECTED: {hazard_type}')
            self.get_logger().warn(f'   Distance: {distance:.2f}m')
            self.get_logger().warn(f'   Location: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})')
            self.get_logger().warn(f'   Image saved: {filename}')
            self.get_logger().warn(f'   Color anomaly: {color_score:.2%}, Edge density: {edge_score:.2%}')
            
            # Trigger emergency stop/path replan
            self.trigger_avoidance()
            
            # Set cooldown to avoid duplicate detections
            self.detection_cooldown = 20  # ~10 seconds
            self.hazard_detected = False
    
    def annotate_hazard_image(self, image, distance, hazard_type, color_score, edge_score):
        """Add annotations to hazard image"""
        # Add red border
        cv2.rectangle(image, (0, 0), (image.shape[1]-1, image.shape[0]-1), 
                     (0, 0, 255), 10)
        
        # Add text overlay
        font = cv2.FONT_HERSHEY_SIMPLEX
        y_offset = 40
        
        cv2.putText(image, f'HAZARD: {hazard_type.upper()}', (20, y_offset), 
                   font, 1.2, (0, 0, 255), 3)
        
        y_offset += 40
        cv2.putText(image, f'Distance: {distance:.2f}m', (20, y_offset), 
                   font, 0.8, (0, 255, 255), 2)
        
        if self.current_pose:
            y_offset += 35
            cv2.putText(image, 
                       f'Pos: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f})', 
                       (20, y_offset), font, 0.7, (0, 255, 255), 2)
        
        y_offset += 35
        cv2.putText(image, f'Color: {color_score:.1%} | Edge: {edge_score:.1%}', 
                   (20, y_offset), font, 0.7, (255, 255, 0), 2)
        
        # Add timestamp
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        y_offset += 35
        cv2.putText(image, timestamp, (20, y_offset), 
                   font, 0.6, (255, 255, 255), 2)
        
        return image
    
    def create_alert_message(self, filepath, distance, hazard_type, color_score, edge_score):
        """Create JSON alert message for control panel"""
        alert = {
            'timestamp': datetime.now().isoformat(),
            'hazard_type': hazard_type,
            'distance': float(distance),
            'image_path': filepath,
            'detection_id': self.detection_count,
            'anomaly_scores': {
                'color': float(color_score),
                'edge_density': float(edge_score)
            }
        }
        
        if self.current_pose:
            alert['location'] = {
                'x': float(self.current_pose.position.x),
                'y': float(self.current_pose.position.y),
                'z': float(self.current_pose.position.z)
            }
        
        return alert
    
    def trigger_avoidance(self):
        """Trigger path replanning by publishing stop command"""
        # Publish stop command
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_override_pub.publish(stop_cmd)
        
        self.get_logger().info('⚠️  Hazard avoidance triggered - Nav2 will replan path')
    
    def detection_loop(self):
        """Periodic detection check"""
        # This runs continuously to monitor for hazards
        pass


def main(args=None):
    rclpy.init(args=args)
    detector = HazardDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
