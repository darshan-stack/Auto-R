#!/usr/bin/env python3
"""
Frontier-based Exploration for Complete Habitat Mapping
Explores new areas automatically without repeating paths
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import numpy as np
from scipy.ndimage import binary_dilation
import math


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # State variables
        self.map_data = None
        self.current_pose = None
        self.exploring = False
        self.visited_frontiers = []
        self.exploration_complete = False
        
        # Parameters
        self.frontier_min_size = 20  # Minimum frontier size in cells
        self.exploration_radius = 10.0  # Maximum exploration distance (meters)
        self.goal_tolerance = 0.5  # Distance to consider goal reached
        
        self.get_logger().info('🗺️  Frontier Explorer Started - Full Habitat Mapping Mode')
        self.get_logger().info('📍 Waiting for map and odometry data...')
        
        # Timer to check for new frontiers
        self.timer = self.create_timer(5.0, self.exploration_cycle)
        
    def odom_callback(self, msg):
        """Update current robot position"""
        self.current_pose = msg.pose.pose
        
    def map_callback(self, msg):
        """Update map data"""
        self.map_data = msg
        
    def exploration_cycle(self):
        """Main exploration loop - find and navigate to frontiers"""
        if self.exploring or not self.map_data or not self.current_pose:
            return
            
        if self.exploration_complete:
            self.get_logger().info('✅ Exploration complete! Full habitat mapped.')
            return
        
        # Find frontiers (boundaries between known and unknown space)
        frontiers = self.find_frontiers()
        
        if not frontiers:
            self.get_logger().info('🎉 No more frontiers found - Exploration Complete!')
            self.get_logger().info('📊 Full habitat map generated successfully')
            self.exploration_complete = True
            return
        
        # Select best frontier to explore
        target_frontier = self.select_best_frontier(frontiers)
        
        if target_frontier:
            self.get_logger().info(f'🎯 New frontier detected at ({target_frontier[0]:.2f}, {target_frontier[1]:.2f})')
            self.navigate_to_frontier(target_frontier)
        
    def find_frontiers(self):
        """Find frontier cells (boundaries between free and unknown space)"""
        if not self.map_data:
            return []
        
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin
        
        # Convert occupancy grid to numpy array
        grid = np.array(self.map_data.data).reshape((height, width))
        
        # Define cell types: -1=unknown, 0=free, 100=occupied
        free_cells = (grid == 0)
        unknown_cells = (grid == -1)
        
        # Dilate free cells to find edges
        dilated_free = binary_dilation(free_cells, iterations=1)
        
        # Frontiers are where dilated free cells meet unknown cells
        frontier_cells = dilated_free & unknown_cells
        
        # Find connected frontier regions
        frontiers = []
        visited = np.zeros_like(frontier_cells, dtype=bool)
        
        for y in range(height):
            for x in range(width):
                if frontier_cells[y, x] and not visited[y, x]:
                    # Flood fill to find connected frontier region
                    region = self.flood_fill(frontier_cells, visited, x, y)
                    
                    if len(region) >= self.frontier_min_size:
                        # Calculate centroid of frontier region
                        centroid_x = sum(r[0] for r in region) / len(region)
                        centroid_y = sum(r[1] for r in region) / len(region)
                        
                        # Convert grid coordinates to world coordinates
                        world_x = origin.position.x + centroid_x * resolution
                        world_y = origin.position.y + centroid_y * resolution
                        
                        # Check if within exploration radius
                        dist = self.distance_to_robot(world_x, world_y)
                        if dist <= self.exploration_radius:
                            frontiers.append((world_x, world_y, len(region), dist))
        
        self.get_logger().info(f'🔍 Found {len(frontiers)} frontier regions')
        return frontiers
    
    def flood_fill(self, grid, visited, start_x, start_y):
        """Flood fill to find connected frontier cells"""
        stack = [(start_x, start_y)]
        region = []
        
        while stack:
            x, y = stack.pop()
            
            if x < 0 or x >= grid.shape[1] or y < 0 or y >= grid.shape[0]:
                continue
            if visited[y, x] or not grid[y, x]:
                continue
            
            visited[y, x] = True
            region.append((x, y))
            
            # Check 4-connected neighbors
            stack.extend([(x+1, y), (x-1, y), (x, y+1), (x, y-1)])
        
        return region
    
    def distance_to_robot(self, x, y):
        """Calculate distance from robot to a point"""
        if not self.current_pose:
            return float('inf')
        
        dx = x - self.current_pose.position.x
        dy = y - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def select_best_frontier(self, frontiers):
        """Select the best frontier to explore based on size and distance"""
        if not frontiers:
            return None
        
        # Filter out previously visited frontiers
        new_frontiers = []
        for frontier in frontiers:
            is_new = True
            for visited in self.visited_frontiers:
                dist = math.sqrt((frontier[0]-visited[0])**2 + (frontier[1]-visited[1])**2)
                if dist < 1.0:  # Too close to previously visited frontier
                    is_new = False
                    break
            if is_new:
                new_frontiers.append(frontier)
        
        if not new_frontiers:
            self.get_logger().info('⚠️  All nearby frontiers already visited')
            return None
        
        # Score frontiers: prefer larger frontiers that are closer
        # Score = size / distance
        best_frontier = max(new_frontiers, key=lambda f: f[2] / max(f[3], 0.1))
        
        # Mark as visited
        self.visited_frontiers.append((best_frontier[0], best_frontier[1]))
        
        return (best_frontier[0], best_frontier[1])
    
    def navigate_to_frontier(self, target):
        """Send navigation goal to frontier"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Navigation server not available')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target[0]
        goal_msg.pose.pose.position.y = target[1]
        goal_msg.pose.pose.position.z = 0.0
        
        # Face towards the frontier
        if self.current_pose:
            dx = target[0] - self.current_pose.position.x
            dy = target[1] - self.current_pose.position.y
            yaw = math.atan2(dy, dx)
            
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal_msg.pose.pose.orientation.w = 1.0
        
        self.exploring = True
        self.get_logger().info(f'🚀 Navigating to new frontier at ({target[0]:.2f}, {target[1]:.2f})')
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle navigation goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('⚠️  Goal rejected by navigation server')
            self.exploring = False
            return
        
        self.get_logger().info('✅ Frontier goal accepted - exploring new area...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        if self.current_pose:
            dx = feedback.current_pose.pose.position.x - self.current_pose.position.x
            dy = feedback.current_pose.pose.position.y - self.current_pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            # Log progress every 3 meters
    
    def result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.exploring = False
        
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('🎉 Frontier reached - new area mapped!')
        elif status == 5:  # ABORTED
            self.get_logger().warn('⚠️  Navigation aborted - trying next frontier')
        elif status == 6:  # CANCELED
            self.get_logger().warn('⚠️  Navigation canceled')
        else:
            self.get_logger().warn(f'⚠️  Navigation ended with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    explorer = FrontierExplorer()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
