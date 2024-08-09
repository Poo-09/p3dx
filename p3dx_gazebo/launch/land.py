import sys
import rospy
from std_msgs.msg import Empty

def land_drone(land_pub):
    land_msg = Empty()
    land_pub.publish(land_msg)
    rospy.loginfo("Land command published")

def main(num_drones):
    rospy.init_node('land_node', anonymous=True)
    
    # Create publishers for all drones
    land_pubs = [rospy.Publisher(f'/ardrone{i}/land', Empty, queue_size=10) for i in range(1, num_drones + 1)]
    
    rospy.sleep(1)  # Give some time for the publishers to set up
    
    # Send land command to each drone
    for land_pub in land_pubs:
        land_drone(land_pub)
        rospy.sleep(2)  # Wait to ensure the command is sent

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 land.py <num_drones>")
        sys.exit(1)

    num_drones = int(sys.argv[1])
    
    try:
        main(num_drones)
    except rospy.ROSInterruptException:
        pass

