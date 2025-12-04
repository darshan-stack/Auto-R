#!/usr/bin/env python3
"""
Environmental Monitoring System for LunaBot
Monitors habitat environmental parameters (temperature, O2, pressure)
for ISRO Lunar Habitat Autonomous Navigation Challenge
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
import random

class EnvironmentMonitor(Node):
    def __init__(self):
        super().__init__('environment_monitor')
        
        # Publishers for environmental parameters
        self.temp_pub = self.create_publisher(Float32, '/habitat/temperature', 10)
        self.o2_pub = self.create_publisher(Float32, '/habitat/oxygen_level', 10)
        self.pressure_pub = self.create_publisher(Float32, '/habitat/pressure', 10)
        self.alert_pub = self.create_publisher(String, '/habitat/alerts', 10)
        self.status_pub = self.create_publisher(String, '/habitat/system_status', 10)
        
        # Subscribe to robot velocity to detect activity
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Monitoring timer (2 Hz)
        self.timer = self.create_timer(0.5, self.monitor_callback)
        
        # State variables
        self.robot_active = False
        self.alert_count = 0
        self.cycle_count = 0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🌡️  LunaBot Environment Monitor System Started')
        self.get_logger().info('📊 Monitoring: Temperature, O2 Level, Pressure')
        self.get_logger().info('🤖 Robot Activity Tracking: Enabled')
        self.get_logger().info('=' * 60)
    
    def cmd_vel_callback(self, msg):
        """Detect if robot is moving"""
        self.robot_active = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01
    
    def monitor_callback(self):
        """Monitor environmental parameters"""
        self.cycle_count += 1
        
        # Simulate environmental parameters
        # Temperature (ideal: 20-24°C for lunar habitat)
        base_temp = 22.0
        temp_variation = 2.0 if self.robot_active else 1.0
        temperature = base_temp + random.uniform(-temp_variation, temp_variation)
        
        # O2 level (ideal: 19.5-23.5%)
        base_o2 = 21.0
        o2_variation = 1.5 if self.robot_active else 0.5
        oxygen = base_o2 + random.uniform(-o2_variation, o2_variation)
        
        # Pressure (ideal: 101.3 kPa)
        base_pressure = 101.3
        pressure = base_pressure + random.uniform(-2.0, 2.0)
        
        # Publish sensor data
        temp_msg = Float32()
        temp_msg.data = temperature
        self.temp_pub.publish(temp_msg)
        
        o2_msg = Float32()
        o2_msg.data = oxygen
        self.o2_pub.publish(o2_msg)
        
        pressure_msg = Float32()
        pressure_msg.data = pressure
        self.pressure_pub.publish(pressure_msg)
        
        # Check thresholds and generate alerts
        alerts = []
        status = "NOMINAL"
        
        if temperature < 18.0 or temperature > 26.0:
            alerts.append(f"⚠️  TEMP ALERT: {temperature:.1f}°C (Normal: 20-24°C)")
            status = "WARNING"
        
        if oxygen < 19.5 or oxygen > 23.5:
            alerts.append(f"⚠️  O2 ALERT: {oxygen:.1f}% (Normal: 19.5-23.5%)")
            status = "WARNING"
        
        if pressure < 99.0 or pressure > 103.0:
            alerts.append(f"⚠️  PRESSURE ALERT: {pressure:.1f}kPa (Normal: 99-103kPa)")
            status = "WARNING"
        
        # Publish alerts if any
        if alerts:
            self.alert_count += 1
            alert_msg = String()
            alert_msg.data = " | ".join(alerts)
            self.alert_pub.publish(alert_msg)
            for alert in alerts:
                self.get_logger().warn(alert)
        else:
            if self.alert_count > 0:
                self.get_logger().info("✅ All parameters returned to nominal range")
                self.alert_count = 0
        
        # Publish system status
        robot_state = "ACTIVE" if self.robot_active else "IDLE"
        status_msg = String()
        status_msg.data = (
            f"Status: {status} | "
            f"Temp: {temperature:.1f}°C | "
            f"O2: {oxygen:.1f}% | "
            f"Press: {pressure:.1f}kPa | "
            f"Robot: {robot_state}"
        )
        self.status_pub.publish(status_msg)
        
        # Log every 10 cycles (5 seconds)
        if self.cycle_count % 10 == 0:
            self.get_logger().info(f"📊 {status_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    monitor = EnvironmentMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.get_logger().info('Shutting down Environment Monitor...')
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
