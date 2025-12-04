#!/usr/bin/env python3
"""
Path Tracking and Footprint Recording System
Records robot's traversed path with coordinates and timestamps
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import os
from datetime import datetime
import math


class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/traveled_path', 10)
        self.footprint_pub = self.create_publisher(String, '/path_footprint', 10)
        
        # Path tracking
        self.path_points = []
        self.last_recorded_pose = None
        self.recording_interval = 0.5  # meters between recorded points
        self.total_distance = 0.0
        
        # Files for logging
        self.log_dir = os.path.expanduser('~/lunabot_path_logs')
        os.makedirs(self.log_dir, exist_ok=True)
        
        self.session_start = datetime.now()
        self.session_id = self.session_start.strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(self.log_dir, f'path_{self.session_id}.json')
        self.csv_file = os.path.join(self.log_dir, f'path_{self.session_id}.csv')
        
        # Initialize CSV file
        with open(self.csv_file, 'w') as f:
            f.write('timestamp,x,y,z,orientation_z,orientation_w,distance_traveled\n')
        
        # Timer for periodic footprint publishing
        self.timer = self.create_timer(2.0, self.publish_footprint)
        
        self.get_logger().info('📍 Path Tracker Started')
        self.get_logger().info(f'📁 Logging to: {self.log_dir}')
        self.get_logger().info(f'📝 Session ID: {self.session_id}')
        
    def odom_callback(self, msg):
        """Record robot position at intervals"""
        current_pose = msg.pose.pose
        
        # Check if we should record this point
        if self.should_record_point(current_pose):
            self.record_point(current_pose, msg.header.stamp)
            
    def should_record_point(self, pose):
        """Determine if this point should be recorded based on distance traveled"""
        if self.last_recorded_pose is None:
            return True
        
        # Calculate distance from last recorded point
        dx = pose.position.x - self.last_recorded_pose.position.x
        dy = pose.position.y - self.last_recorded_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance >= self.recording_interval
    
    def record_point(self, pose, timestamp):
        """Record a path point"""
        # Calculate distance from last point
        if self.last_recorded_pose is not None:
            dx = pose.position.x - self.last_recorded_pose.position.x
            dy = pose.position.y - self.last_recorded_pose.position.y
            self.total_distance += math.sqrt(dx*dx + dy*dy)
        
        # Create point record
        point = {
            'timestamp': datetime.now().isoformat(),
            'ros_time': timestamp.sec + timestamp.nanosec * 1e-9,
            'position': {
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'z': float(pose.position.z)
            },
            'orientation': {
                'x': float(pose.orientation.x),
                'y': float(pose.orientation.y),
                'z': float(pose.orientation.z),
                'w': float(pose.orientation.w)
            },
            'distance_traveled': float(self.total_distance)
        }
        
        self.path_points.append(point)
        self.last_recorded_pose = pose
        
        # Write to CSV
        with open(self.csv_file, 'a') as f:
            f.write(f"{point['timestamp']},"
                   f"{point['position']['x']:.4f},"
                   f"{point['position']['y']:.4f},"
                   f"{point['position']['z']:.4f},"
                   f"{point['orientation']['z']:.4f},"
                   f"{point['orientation']['w']:.4f},"
                   f"{point['distance_traveled']:.4f}\n")
        
        # Save JSON periodically (every 10 points)
        if len(self.path_points) % 10 == 0:
            self.save_json_log()
            
        # Publish Path message for visualization
        self.publish_path()
        
    def publish_path(self):
        """Publish path for RViz visualization"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for point in self.path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = point['position']['x']
            pose_stamped.pose.position.y = point['position']['y']
            pose_stamped.pose.position.z = point['position']['z']
            pose_stamped.pose.orientation.z = point['orientation']['z']
            pose_stamped.pose.orientation.w = point['orientation']['w']
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)
    
    def publish_footprint(self):
        """Publish path footprint summary"""
        if not self.path_points:
            return
        
        footprint = {
            'session_id': self.session_id,
            'session_start': self.session_start.isoformat(),
            'total_points': len(self.path_points),
            'total_distance': float(self.total_distance),
            'current_position': self.path_points[-1]['position'] if self.path_points else None,
            'path_summary': {
                'start': self.path_points[0]['position'] if self.path_points else None,
                'end': self.path_points[-1]['position'] if self.path_points else None,
                'waypoints': [p['position'] for p in self.path_points[::10]]  # Every 10th point
            }
        }
        
        self.footprint_pub.publish(String(data=json.dumps(footprint, indent=2)))
        
        # Log summary
        if len(self.path_points) % 50 == 0:  # Log every 50 points
            self.get_logger().info(f'📊 Path Statistics:')
            self.get_logger().info(f'   Points recorded: {len(self.path_points)}')
            self.get_logger().info(f'   Distance traveled: {self.total_distance:.2f}m')
            if self.path_points:
                self.get_logger().info(f'   Current position: ({self.path_points[-1]["position"]["x"]:.2f}, '
                                     f'{self.path_points[-1]["position"]["y"]:.2f})')
    
    def save_json_log(self):
        """Save complete path log to JSON file"""
        log_data = {
            'session_id': self.session_id,
            'session_start': self.session_start.isoformat(),
            'total_points': len(self.path_points),
            'total_distance': float(self.total_distance),
            'path': self.path_points
        }
        
        with open(self.log_file, 'w') as f:
            json.dump(log_data, f, indent=2)
    
    def shutdown(self):
        """Save final log on shutdown"""
        self.save_json_log()
        self.get_logger().info('📊 Final Path Statistics:')
        self.get_logger().info(f'   Total points: {len(self.path_points)}')
        self.get_logger().info(f'   Total distance: {self.total_distance:.2f}m')
        self.get_logger().info(f'   Log saved: {self.log_file}')
        self.get_logger().info(f'   CSV saved: {self.csv_file}')


def main(args=None):
    rclpy.init(args=args)
    tracker = PathTracker()
    
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.shutdown()
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
