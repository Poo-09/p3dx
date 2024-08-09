#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_camera():
    rospy.init_node('camera_publisher', anonymous=True)
    image_pub = rospy.Publisher('/robot1/rgbd_camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    # OpenCV to capture video
    cap = cv2.VideoCapture(0)  # Change the argument to the correct camera index or video file

    if not cap.isOpened():
        rospy.logerr("Failed to open camera.")
        return

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_pub.publish(image_message)
        rate.sleep()
	cap.release()

if _name_ == '_main_':
    try:
        publish_camera()
    except rospy.ROSInterruptException:
        pass
