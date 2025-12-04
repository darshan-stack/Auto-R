#!/usr/bin/env python3
"""
Hazard Detection and Anomaly Detection System with Path Replanning
Detects hazardous/anomalous objects during patrol, captures images,
sends alerts to control panel, and triggers autonomous path replanning
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
from datetime import datetime
import math


class HazardDetector(Node):
    def __init__(self):
        super().__init__('hazard_detector')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Navigation action client for path replanning
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
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
        self.path_sub = self.create_subscription(
            Path, '/traveled_path',
            self.path_callback, 10)
        
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
        self.current_path = []
        self.avoiding_hazard = False
        
        # Detection parameters
        self.hazard_distance_threshold = 2.0  # meters - increased for better reaction time
        self.anomaly_threshold = 0.7  # confidence threshold
        self.image_save_dir = os.path.expanduser('~/lunabot_hazards')
        
        # Create hazard image directory
        os.makedirs(self.image_save_dir, exist_ok=True)
        
        # Simple anomaly detection parameters
        self.baseline_intensity = None
    def odom_callback(self, msg):
        """Update current robot position"""
        self.current_pose = msg.pose.pose
        
    def camera_callback(self, msg):
        """Process camera images"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Camera conversion error: {e}')
    
    def path_callback(self, msg):
        """Update current path from path tracker"""
        self.current_path = [(pose.pose.position.x, pose.pose.position.y) 
                           for pose in msg.poses]s enabled')
        
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
        
    def detect_hazard(self, distance, hazard_type):
        """Detect and report hazard with enhanced logging and path replanning"""
        if self.hazard_detected or self.latest_image is None:
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
            
            # Annotate image with enhanced information
            annotated_image = self.annotate_hazard_image(
                self.latest_image.copy(), distance, hazard_type, color_score, edge_score)
            
            cv2.imwrite(filepath, annotated_image)
            
            # Create comprehensive alert message
            alert = self.create_alert_message(
                filepath, distance, hazard_type, color_score, edge_score)
            
            # Save to hazard log
            self.hazard_log.append(alert)
            self.save_hazard_log()
            
            # Publish alert to control panel
            self.hazard_alert_pub.publish(String(data=json.dumps(alert)))
            
            # Enhanced logging
            self.get_logger().warn('╔════════════════════════════════════════════════════════════╗')
            self.get_logger().warn('║         🚨 HAZARD DETECTED - INITIATING AVOIDANCE          ║')
            self.get_logger().warn('╚════════════════════════════════════════════════════════════╝')
            self.get_logger().warn(f'  Type: {hazard_type.upper()}')
            self.get_logger().warn(f'  Distance: {distance:.2f} meters')
            self.get_logger().warn(f'  GPS Coordinates: ({self.current_pose.position.x:.4f}, {self.current_pose.position.y:.4f})')
            self.get_logger().warn(f'  Detection #: {self.detection_count}')
            self.get_logger().warn(f'  Image: {filename}')
            self.get_logger().warn(f'  Anomaly Scores - Color: {color_score:.2%} | Edge: {edge_score:.2%}')
            self.get_logger().warn(f'  Timestamp: {timestamp}')
            self.get_logger().warn('  Action: Stopping robot and replanning path...')
            self.get_logger().warn('═══════════════════════════════════════════════════════════')
            
            # Trigger emergency stop and path replanning
            self.trigger_avoidance_and_replan(distance, hazard_type)
            
            # Set cooldown to avoid duplicate detections
            self.detection_cooldown = 40  # ~20 seconds
            self.hazard_detected = Falseh replan
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
    def create_alert_message(self, filepath, distance, hazard_type, color_score, edge_score):
        """Create comprehensive JSON alert message for control panel"""
        alert = {
            'timestamp': datetime.now().isoformat(),
            'detection_id': self.detection_count,
            'hazard_type': hazard_type,
            'distance_meters': float(distance),
            'image_path': filepath,
            'image_filename': os.path.basename(filepath),
            'anomaly_scores': {
                'color_anomaly': float(color_score),
                'edge_density': float(edge_score),
                'combined_score': float((color_score + edge_score) / 2)
            },
            'path_info': {
                'current_path_length': len(self.current_path),
                'path_replanning': 'initiated' if self.avoiding_hazard else 'pending'
            }
        }
        
        if self.current_pose:
            alert['gps_coordinates'] = {
                'x': float(self.current_pose.position.x),
    def trigger_avoidance_and_replan(self, distance, hazard_type):
        """
        Trigger emergency stop and intelligent path replanning
        Nav2 will automatically replan around the obstacle
        """
        # Immediate stop command
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_override_pub.publish(stop_cmd)
        
        self.avoiding_hazard = True
        self.get_logger().info('🛑 Emergency stop initiated')
        self.get_logger().info('🔄 Nav2 costmap will mark obstacle and replan path automatically')
        
        # Calculate alternative waypoint (move perpendicular to hazard)
        if self.current_pose:
            # Get current heading
            qz = self.current_pose.orientation.z
            qw = self.current_pose.orientation.w
            current_yaw = 2 * math.atan2(qz, qw)
            
            # Move perpendicular to avoid obstacle (90 degrees right)
            avoid_yaw = current_yaw + math.pi / 2
            
            # Calculate avoidance point 3 meters to the side
            avoid_x = self.current_pose.position.x + 3.0 * math.cos(avoid_yaw)
            avoid_y = self.current_pose.position.y + 3.0 * math.sin(avoid_yaw)
            
            self.get_logger().info(f'📍 Suggested avoidance point: ({avoid_x:.2f}, {avoid_y:.2f})')
            self.get_logger().info('   Nav2 will use costmap to find optimal path around hazard')
        
        # Note: Nav2's local planner will automatically avoid the obstacle
        # detected by the laser scan in the costmap. No manual goal needed.
        # The current navigation goal will be replanned automatically.
        
        self.avoiding_hazard = False
    
    def save_hazard_log(self):
        """Save hazard detection log to JSON file"""
        log_data = {
            'total_detections': len(self.hazard_log),
            'session_start': datetime.now().isoformat(),
            'detections': self.hazard_log
        }
        
        with open(self.hazard_log_file, 'w') as f:
            json.dump(log_data, f, indent=2)
        
        self.get_logger().debug(f'💾 Hazard log updated: {self.hazard_log_file}')
        
        return alerte_density': float(edge_score)
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
