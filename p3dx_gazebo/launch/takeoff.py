import sys
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped

def takeoff_drone(drone_ns, height):
    rospy.init_node('takeoff_node', anonymous=True)
    takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
    pose_pub = rospy.Publisher(f'/{drone_ns}/command/pose', PoseStamped, queue_size=10)
    
    rospy.loginfo(f"Initializing takeoff for {drone_ns} to height {height}m")
    
    rospy.sleep(1)  # Give some time for the publisher to set up
    
    # Publish takeoff message
    takeoff_msg = Empty()
    takeoff_pub.publish(takeoff_msg)
    rospy.loginfo("Published takeoff command")
    
    rospy.sleep(1)
    
    # Publish pose message
    pose_msg = PoseStamped()
    pose_msg.pose.position.z = height
    pose_pub.publish(pose_msg)
    rospy.loginfo(f"Published pose command to {drone_ns} to height {height}m")
    
    rospy.sleep(5)  # Wait for the drone to reach the height

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 takeoff.py <num_drones> <takeoff_height>")
        sys.exit(1)
    
    num_drones = int(sys.argv[1])
    takeoff_height = float(sys.argv[2])
    
    for i in range(1, num_drones + 1):
        drone_ns = f"ardrone{i}"
        try:
            takeoff_drone(drone_ns, takeoff_height)
        except Exception as e:
            rospy.logerr(f"Error taking off drone {drone_ns}: {e}")

