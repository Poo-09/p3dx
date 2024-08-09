#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from threading import Thread
import math

def move_in_circle(namespace, linear_speed, radius, phase_offset):
    velocity_publisher = rospy.Publisher(f'/{namespace}/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now()

    # Calculate the total time to complete one circle
    circle_time = (2 * math.pi * radius) / linear_speed

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = (current_time - start_time).to_sec()
        
        if elapsed_time > circle_time:
            # Stop the robot after completing one circle
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            break

        # Calculate the position on the circle with the phase offset
        angle = linear_speed * elapsed_time / radius + phase_offset
        
        # Update the velocities to keep the robot on the circle path
        vel_msg.linear.x = linear_speed * math.cos(angle)
        vel_msg.linear.y = linear_speed * math.sin(angle)
        vel_msg.angular.z = linear_speed / radius
        
        velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('multi_robot_circle_move', anonymous=True)
        
        # Define the namespaces for the robots
        namespaces = ['robot1', 'robot2', 'robot3', 'robot4']
        
        # Linear speed and radius
        linear_speed = 0.2
        radius = 2.0  # Radius of the circle
        
        # Create and start threads for each robot
        threads = []
        for i, ns in enumerate(namespaces):
            phase_offset = (2 * math.pi * i) / len(namespaces)  # Phase offset for each robot
            t = Thread(target=move_in_circle, args=(ns, linear_speed, radius, phase_offset))
            t.start()
            threads.append(t)
        
        # Wait for all threads to complete
        for t in threads:
            t.join()
    except rospy.ROSInterruptException:
        pass

