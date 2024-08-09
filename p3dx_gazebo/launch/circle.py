import sys
import rospy
from geometry_msgs.msg import Twist
import math

def move_in_circle(drone_ns, radius):
    rospy.init_node(f'circle_movement_node_{drone_ns}', anonymous=True)
    vel_pub = rospy.Publisher(f'/{drone_ns}/cmd_vel', Twist, queue_size=10)
    
    vel_msg = Twist()
    vel_msg.linear.x = radius
    vel_msg.angular.z = 0.2 
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        vel_pub.publish(vel_msg)
        rate.sleep()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 circle.py <num_drones> <radius>")
        sys.exit(1)
    
    num_drones = int(sys.argv[1])
    radius = float(sys.argv[2])
    
    for i in range(1, num_drones + 1):
        drone_ns = f"ardrone{i}"
        move_in_circle(drone_ns, radius)

