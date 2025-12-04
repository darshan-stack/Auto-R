#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import random

class LunarMission(Node):
    def __init__(self):
        super().__init__('lunar_mission_node')
        self.o2_pub = self.create_publisher(Float32, '/lunar/o2_level', 10)
        self.timer = self.create_timer(2.0, self.publish_telemetry)

    def publish_telemetry(self):
        o2 = random.uniform(19.0, 21.0)
        self.o2_pub.publish(Float32(data=o2))
        self.get_logger().info(f"O2: {o2:.2f}%")

def create_pose(navigator, x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()

    # Wait for Nav2 activation
    nav.waitUntilNav2Active()
    mission = LunarMission()

    p1 = create_pose(nav, 2.0, 2.0)
    p2 = create_pose(nav, 5.0, 5.0)
    p3 = create_pose(nav, 0.0, 0.0)
    waypoints = [p1, p2, p3]

    nav.followWaypoints(waypoints)

    while not nav.isTaskComplete():
        rclpy.spin_once(mission, timeout_sec=1.0)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

