#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

def takeoff():
    pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
    pub_takeoff.publish(Empty())
    rospy.loginfo("Takeoff command sent.")

def land():
    pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
    pub_land.publish(Empty())
    rospy.loginfo("Land command sent.")

def hover():
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    hover_cmd = Twist()
    pub_cmd_vel.publish(hover_cmd)
    rospy.loginfo("Hover command sent.")

if __name__ == '__main__':
    rospy.init_node('drone_control')
    takeoff()
    time.sleep(5)  # Hover for 5 seconds
    hover()
    time.sleep(5)  # Hover for 5 seconds
    land()

