#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from threading import Thread
import math

def move_in_circle(namespace, linear_speed, radius, delay):
    velocity_publisher = rospy.Publisher(f'/{namespace}/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    rate = rospy.Rate(10)
    start_time = rospy.Time.now()

    rospy.sleep(delay)  

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = (current_time - start_time).to_sec()
        
        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = linear_speed / radius
        
        velocity_publisher.publish(vel_msg)
        rate.sleep()
    vel_msg.linear.x=0
    vel_msg.angular.z=0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
       
        rospy.init_node('multi_robot_circle_move', anonymous=True)
        
        
        namespaces = ['robot1', 'robot2', 'robot3', 'robot4']
        
        linear_speed = 0.2
        radius = 2.0 
        
        delays = [0, 1, 2, 3]  
       
        threads = []
        for ns, delay in zip(namespaces, delays):
            t = Thread(target=move_in_circle, args=(ns, linear_speed, radius, delay))
            t.start()
            threads.append(t)
        
        for t in threads:
            t.join()
    except rospy.ROSInterruptException:
        pass
