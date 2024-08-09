#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import sys, select, termios, tty

msg = """
Control Your AR Drone!
---------------------------
Moving around:
        i
   j    k    l

   y - up
   h - down

   u - counterclockwise
   o - clockwise

t : takeoff
g : land

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'k': (-1, 0, 0, 0),
    'j': (0, 1, 0, 0),
    'l': (0, -1, 0, 0),
    'y': (0, 0, 1, 0),
    'h': (0, 0, -1, 0),
    'u': (0, 0, 0, 1),
    'o': (0, 0, 0, -1),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('ardrone_teleop')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
    land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)

    x = 0
    y = 0
    z = 0
    th = 0

    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key == 't':
                takeoff_pub.publish(Empty())
            elif key == 'g':
                land_pub.publish(Empty())
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x
            twist.linear.y = y
            twist.linear.z = z
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
