#!/usr/bin/env python3
"""
Autonomous Patrol Node for ISRO LunaBot Challenge
Performs continuous autonomous navigation with predefined waypoints
Industry-ready implementation with error handling and monitoring
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import time

class AutonomousPatrol(Node):
    def __init__(self):
        super().__init__('autonomous_patrol')
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        
        # Patrol waypoints (x, y, yaw in radians)
        self.waypoints = [
            (2.0, 0.0, 0.0),      # Forward 2m
            (2.0, 2.0, 1.57),     # Turn left, forward 2m
            (0.0, 2.0, 3.14),     # Turn left, forward 2m
            (0.0, 0.0, -1.57),    # Turn left, return to start
        ]
        
        self.current_waypoint_index = 0
        self.mission_running = False
        
        self.get_logger().info('🤖 Autonomous Patrol Node Started')
        self.get_logger().info(f'📍 Loaded {len(self.waypoints)} waypoints')
        
        # Wait for navigation server
        self.timer = self.create_timer(2.0, self.check_nav_server)
        
    def check_nav_server(self):
        """Check if navigation server is available and start mission"""
        if self.nav_client.wait_for_server(timeout_sec=1.0):
            self.timer.cancel()
            self.get_logger().info('✅ Navigation server connected!')
            self.get_logger().info('🚀 Starting autonomous patrol mission in 3 seconds...')
            time.sleep(3)
            self.start_patrol()
        else:
            self.get_logger().warn('⏳ Waiting for navigation server...')
    
    def start_patrol(self):
        """Start autonomous patrol"""
        self.mission_running = True
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎯 AUTONOMOUS PATROL MISSION STARTED')
        self.get_logger().info('=' * 60)
        self.send_next_goal()
    
    def create_pose(self, x, y, yaw):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose
    
    def send_next_goal(self):
        """Send next waypoint goal"""
        if not self.mission_running:
            return
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('=' * 60)
            self.get_logger().info('✅ PATROL MISSION COMPLETED!')
            self.get_logger().info(f'📊 Visited {len(self.waypoints)} waypoints successfully')
            self.get_logger().info('=' * 60)
            # Loop back to start
            self.current_waypoint_index = 0
            time.sleep(5)  # Wait 5 seconds before restarting
        
        waypoint = self.waypoints[self.current_waypoint_index]
        x, y, yaw = waypoint
        
        self.get_logger().info('')
        self.get_logger().info(f'📍 Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}')
        self.get_logger().info(f'   Target: X={x:.2f}m, Y={y:.2f}m, Yaw={math.degrees(yaw):.1f}°')
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(x, y, yaw)
        
        # Send goal
        self.get_logger().info('🚀 Sending navigation goal...')
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Publish status
        status_msg = String()
        status_msg.data = f'Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}'
        self.status_pub.publish(status_msg)
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose.pose
        distance_remaining = feedback.distance_remaining
        
        # Log progress every few seconds
        if hasattr(self, '_last_log_time'):
            if (self.get_clock().now().nanoseconds - self._last_log_time) < 3e9:
                return
        
        self._last_log_time = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f'📊 Progress: {distance_remaining:.2f}m remaining | '
            f'Pos: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f})'
        )
    
    def goal_response_callback(self, future):
        """Handle goal acceptance response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by navigation server!')
            return
        
        self.get_logger().info('✅ Goal accepted! Robot moving...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('🎉 Waypoint reached successfully!')
            self.current_waypoint_index += 1
            
            # Send next goal after short delay
            time.sleep(1)
            self.send_next_goal()
        else:
            self.get_logger().error(f'⚠️ Navigation failed with status: {status}')
            self.get_logger().warn('🔄 Retrying current waypoint in 3 seconds...')
            time.sleep(3)
            self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousPatrol()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Autonomous patrol stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
