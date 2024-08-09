#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math
import sys

class CircleMovement:
    def __init__(self, num_robots, radius):
        self.num_robots = num_robots
        self.radius = radius
        self.publishers = []

        for i in range(1, num_robots + 1):
            topic = f'/robot{i}/cmd_vel'
            self.publishers.append(rospy.Publisher(topic, Twist, queue_size=10))

        rospy.init_node('circle_movement', anonymous=True)
        self.rate = rospy.Rate(10)

    def move_in_circle(self):
        while not rospy.is_shutdown():
            for i, pub in enumerate(self.publishers):
                twist = Twist()
                twist.linear.x = 0.2  # Constant linear speed
                twist.angular.z = twist.linear.x / self.radius  # Angular speed to maintain circular path

                pub.publish(twist)

            self.rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: circle_movement.py num_robots radius")
        sys.exit(1)

    num_robots = int(sys.argv[1])
    radius = float(sys.argv[2])
    
    mover = CircleMovement(num_robots, radius)
    mover.move_in_circle()

