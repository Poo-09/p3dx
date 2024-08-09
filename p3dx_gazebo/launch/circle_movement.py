#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math
import sys
import signal

class CircleMovement:
    def __init__(self, num_robots, radius):
        self.num_robots = num_robots
        self.radius = radius
        self.publishers = []
        self.running = True

        for i in range(1, num_robots + 1):
            topic = f'/robot{i}/cmd_vel'
            self.publishers.append(rospy.Publisher(topic, Twist, queue_size=10))

        rospy.init_node('circle_movement', anonymous=True)
        self.rate = rospy.Rate(10)

        # Set up signal handlers
        signal.signal(signal.SIGINT, self.stop_motion)
        signal.signal(signal.SIGTERM, self.stop_motion)

    def stop_motion(self, signum, frame):
        self.running = False
        rospy.loginfo("Stopping motion...")
        # Stop the robots by publishing zero velocities
        for pub in self.publishers:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            pub.publish(twist)
        rospy.signal_shutdown('Terminated')

    def move_in_circle(self):
        rospy.loginfo("Starting circle movement...")
        while self.running and not rospy.is_shutdown():
            for pub in self.publishers:
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
