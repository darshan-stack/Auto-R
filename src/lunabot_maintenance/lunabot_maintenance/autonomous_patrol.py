#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class AutonomousPatrol(Node):
    def __init__(self):
        super().__init__('autonomous_patrol')
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Define patrol waypoints (x, y, yaw)
        self.waypoints = [
            (3.0, 2.0, 0.0),
            (1.0, 4.0, 1.57),
            (-2.0, 1.0, 3.14),
            (0.0, 0.0, 0.0)
        ]
        
        self.current_waypoint = 0
        
        self.get_logger().info('🚀 Autonomous Patrol System Started')
        self.get_logger().info(f'📍 Patrol route: {len(self.waypoints)} waypoints')
        
        # Wait for navigation action server
        self.get_logger().info('⏳ Waiting for navigation action server...')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('✅ Navigation server ready!')
        
        # Start patrol
        self.timer = self.create_timer(2.0, self.patrol_callback)
        
    def patrol_callback(self):
        if self.nav_to_pose_client.server_is_ready():
            self.send_next_goal()
            self.timer.cancel()  # Stop timer, goal callback will continue
    
    def send_next_goal(self):
        x, y, yaw = self.waypoints[self.current_waypoint]
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        from math import sin, cos
        goal_msg.pose.pose.orientation.z = sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2.0)
        
        self.get_logger().info(f'🎯 Sending goal {self.current_waypoint + 1}/{len(self.waypoints)}: ({x:.1f}, {y:.1f})')
        
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected!')
            return
        
        self.get_logger().info('✅ Goal accepted! Robot is navigating...')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        # Log progress occasionally
        pass
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'🏁 Reached waypoint {self.current_waypoint + 1}!')
        
        # Move to next waypoint
        self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
        
        if self.current_waypoint == 0:
            self.get_logger().info('🔄 Patrol loop completed! Starting next round...')
        
        # Wait 2 seconds before next goal
        time.sleep(2.0)
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    patrol = AutonomousPatrol()
    
    try:
        rclpy.spin(patrol)
    except KeyboardInterrupt:
        patrol.get_logger().info('🛑 Patrol stopped by user')
    finally:
        patrol.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
