#!/usr/bin/env python3
"""
Enhanced Hazard Detection System with 3D LiDAR and Precise GPS Tracking
Detects obstacles using PointCloud2 data, captures images with accurate location data
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
from datetime import datetime
import math


class EnhancedHazardDetector(Node):
    def __init__(self):
        super().__init__('enhanced_hazard_detector')
        
        # CV Bridge
        self.bridge = CvBridge()
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribers - Using ACTUAL robot topics
        self.camera_sub = self.create_subscription(
            Image, '/front_stereo_camera/left/image_raw', 
            self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/front_3d_lidar/lidar_points',
            self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/chassis/odom', 
            self.odom_callback, 10)
        
        # Publishers
        self.hazard_alert_pub = self.create_publisher(String, '/hazard_alerts', 10)
        self.cmd_vel_override_pub = self.create_publisher(Twist, '/hazard_stop', 10)
        self.debug_image_pub = self.create_publisher(Image, '/hazard_debug_image', 10)
        
        # State
        self.current_pose = None
        self.latest_image = None
        self.detection_cooldown = 0
        self.detection_count = 0
        
        # Enhanced detection parameters
        self.hazard_distance_threshold = 3.0  # meters
        self.hazard_height_min = -0.2  # ground clearance
        self.hazard_height_max = 2.5   # max obstacle height
        self.min_obstacle_points = 30  # minimum points to confirm
        self.detection_fov = 60  # degrees field of view
        
        # Output directory
        self.image_save_dir = os.path.expanduser('~/lunabot_hazards')
        os.makedirs(self.image_save_dir, exist_ok=True)
        
        self.hazard_log_file = os.path.join(self.image_save_dir, 'hazard_detections.json')
        self.hazard_log = []
        
        # GPS simulation (convert XY to lat/lon)
        self.gps_origin_lat = 0.0
        self.gps_origin_lon = 0.0
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('🚨 ENHANCED HAZARD DETECTION SYSTEM INITIALIZED')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'📡 LiDAR Topic: /front_3d_lidar/lidar_points')
        self.get_logger().info(f'📷 Camera Topic: /front_stereo_camera/left/image_raw')
        self.get_logger().info(f'📍 Odometry Topic: /chassis/odom')
        self.get_logger().info(f'🎯 Detection Range: {self.hazard_distance_threshold}m')
        self.get_logger().info(f'📊 Min Points Required: {self.min_obstacle_points}')
        self.get_logger().info(f'📁 Save Directory: {self.image_save_dir}')
        self.get_logger().info('=' * 70)
        
        # Status timer
        self.create_timer(5.0, self.print_status)
        
    def odom_callback(self, msg):
        """Update robot position"""
        self.current_pose = msg.pose.pose
        
    def camera_callback(self, msg):
        """Process camera images"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')
            
    def lidar_callback(self, msg):
        """Process 3D LiDAR point cloud for obstacle detection"""
        if self.detection_cooldown > 0:
            self.detection_cooldown -= 1
            return
            
        try:
            # Extract points from PointCloud2
            points = []
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = point
                # Filter points in front of robot within detection range
                distance = math.sqrt(x*x + y*y)
                
                if (x > 0 and  # in front
                    distance < self.hazard_distance_threshold and
                    z > self.hazard_height_min and
                    z < self.hazard_height_max):
                    points.append((x, y, z, distance))
            
            if len(points) >= self.min_obstacle_points:
                # Calculate obstacle center and closest point
                points_array = np.array(points)
                min_dist_idx = np.argmin(points_array[:, 3])
                closest_point = points_array[min_dist_idx]
                
                # Calculate obstacle centroid
                centroid_x = np.mean(points_array[:, 0])
                centroid_y = np.mean(points_array[:, 1])
                centroid_z = np.mean(points_array[:, 2])
                
                obstacle_info = {
                    'distance': float(closest_point[3]),
                    'position': {
                        'x': float(centroid_x),
                        'y': float(centroid_y),
                        'z': float(centroid_z)
                    },
                    'num_points': len(points),
                    'closest_point': {
                        'x': float(closest_point[0]),
                        'y': float(closest_point[1]),
                        'z': float(closest_point[2])
                    }
                }
                
                self.detect_hazard(obstacle_info)
                
        except Exception as e:
            self.get_logger().error(f'LiDAR processing error: {e}')
            
    def detect_hazard(self, obstacle_info):
        """Process detected hazard"""
        if not self.current_pose or not self.latest_image is not None:
            return
            
        self.detection_count += 1
        distance = obstacle_info['distance']
        
        # Calculate absolute position
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        robot_z = self.current_pose.position.z
        
        # Get robot orientation
        qz = self.current_pose.orientation.z
        qw = self.current_pose.orientation.w
        robot_yaw = 2 * math.atan2(qz, qw)
        
        # Transform obstacle position to world frame
        obs_local_x = obstacle_info['position']['x']
        obs_local_y = obstacle_info['position']['y']
        obs_local_z = obstacle_info['position']['z']
        
        abs_x = robot_x + obs_local_x * math.cos(robot_yaw) - obs_local_y * math.sin(robot_yaw)
        abs_y = robot_y + obs_local_x * math.sin(robot_yaw) + obs_local_y * math.cos(robot_yaw)
        abs_z = robot_z + obs_local_z
        
        # Convert to GPS coordinates (simulated)
        gps_lat = self.gps_origin_lat + (abs_y / 111320.0)
        gps_lon = self.gps_origin_lon + (abs_x / (111320.0 * math.cos(math.radians(gps_lat))))
        
        # Capture and annotate image
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'hazard_{self.detection_count:04d}_{timestamp}.jpg'
        filepath = os.path.join(self.image_save_dir, filename)
        
        annotated_image = self.annotate_image(
            self.latest_image.copy(), obstacle_info, abs_x, abs_y, abs_z)
        cv2.imwrite(filepath, annotated_image)
        
        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Debug image publish error: {e}')
        
        # Create comprehensive alert
        alert = {
            'timestamp': datetime.now().isoformat(),
            'detection_id': self.detection_count,
            'hazard_type': 'obstacle',
            'distance_meters': distance,
            'image_path': filepath,
            'image_filename': filename,
            'obstacle_info': {
                'num_points': obstacle_info['num_points'],
                'height_meters': float(obstacle_info['position']['z']),
                'width_estimate': 'calculating'
            },
            'robot_position': {
                'x': robot_x,
                'y': robot_y,
                'z': robot_z,
                'yaw_radians': robot_yaw
            },
            'obstacle_absolute_position': {
                'x': abs_x,
                'y': abs_y,
                'z': abs_z
            },
            'gps_coordinates': {
                'latitude': gps_lat,
                'longitude': gps_lon,
                'altitude': abs_z
            },
            'obstacle_relative_position': obstacle_info['position']
        }
        
        # Save to log
        self.hazard_log.append(alert)
        self.save_hazard_log()
        
        # Publish alert
        self.hazard_alert_pub.publish(String(data=json.dumps(alert, indent=2)))
        
        # Log detection
        self.get_logger().warn('╔' + '═' * 68 + '╗')
        self.get_logger().warn('║' + ' ' * 15 + '🚨 HAZARD DETECTED' + ' ' * 34 + '║')
        self.get_logger().warn('╚' + '═' * 68 + '╝')
        self.get_logger().warn(f'  Detection ID: #{self.detection_count}')
        self.get_logger().warn(f'  Distance: {distance:.2f} meters')
        self.get_logger().warn(f'  Points Detected: {obstacle_info["num_points"]}')
        self.get_logger().warn(f'  Robot Position: ({robot_x:.2f}, {robot_y:.2f}, {robot_z:.2f})')
        self.get_logger().warn(f'  Obstacle Position: ({abs_x:.2f}, {abs_y:.2f}, {abs_z:.2f})')
        self.get_logger().warn(f'  GPS: Lat {gps_lat:.6f}, Lon {gps_lon:.6f}')
        self.get_logger().warn(f'  Image: {filename}')
        self.get_logger().warn('  Action: Emergency stop + path replanning')
        self.get_logger().warn('═' * 70)
        
        # Trigger avoidance
        self.trigger_avoidance()
        
        # Cooldown
        self.detection_cooldown = 30
        
    def annotate_image(self, image, obstacle_info, abs_x, abs_y, abs_z):
        """Add detailed annotations to image"""
        # Red border
        cv2.rectangle(image, (0, 0), (image.shape[1]-1, image.shape[0]-1), (0, 0, 255), 15)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        y = 50
        
        # Title
        cv2.putText(image, '🚨 HAZARD DETECTED', (20, y), font, 1.5, (0, 0, 255), 4)
        
        y += 50
        cv2.putText(image, f'Distance: {obstacle_info["distance"]:.2f}m', (20, y), font, 1.0, (0, 255, 255), 3)
        
        y += 45
        cv2.putText(image, f'Points: {obstacle_info["num_points"]}', (20, y), font, 0.9, (0, 255, 255), 2)
        
        y += 45
        cv2.putText(image, f'Position: ({abs_x:.2f}, {abs_y:.2f}, {abs_z:.2f})', (20, y), font, 0.8, (255, 255, 0), 2)
        
        y += 40
        if self.current_pose:
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
            cv2.putText(image, f'Robot: ({robot_x:.2f}, {robot_y:.2f})', (20, y), font, 0.8, (255, 255, 0), 2)
        
        y += 40
        cv2.putText(image, f'Height: {obstacle_info["position"]["z"]:.2f}m', (20, y), font, 0.8, (255, 255, 0), 2)
        
        # Timestamp
        y = image.shape[0] - 20
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        cv2.putText(image, timestamp, (20, y), font, 0.7, (255, 255, 255), 2)
        
        return image
        
    def trigger_avoidance(self):
        """Emergency stop and signal for replanning"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_override_pub.publish(stop_cmd)
        
        self.get_logger().info('🛑 Emergency stop commanded')
        self.get_logger().info('🔄 Nav2 will replan path automatically')
        
    def save_hazard_log(self):
        """Save detection log to JSON"""
        log_data = {
            'total_detections': len(self.hazard_log),
            'session_start': datetime.now().isoformat(),
            'detections': self.hazard_log
        }
        
        with open(self.hazard_log_file, 'w') as f:
            json.dump(log_data, f, indent=2)
            
    def print_status(self):
        """Periodic status update"""
        if self.detection_count > 0:
            self.get_logger().info(f'📊 Status: {self.detection_count} hazards detected, cooldown: {self.detection_cooldown}')
        

def main(args=None):
    rclpy.init(args=args)
    detector = EnhancedHazardDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
