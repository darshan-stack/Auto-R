#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

msg = """
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
      LUNABOT TELEOP CONTROL
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Controls:
   u    i    o
   j    k    l
   m    ,    .

u/o : forward + turn left/right
j/l : turn left/right  
m/. : backward + turn left/right
i   : forward
,   : backward
k   : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease linear speed by 10%
e/c : increase/decrease angular speed by 10%

CTRL-C to quit

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = Node('teleop_keyboard')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    speed = 0.3
    turn = 0.5
    x = 0.0
    th = 0.0
    status = 0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif key == 'k':
                x = 0.0
                th = 0.0
            else:
                if key == '\x03':
                    break

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
