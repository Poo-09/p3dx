#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageCapture:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.image_count = 0
        self.image_path = '/path/to/catkin_ws/src/camera_calibration/calibration_images/'
        rospy.loginfo("Image capture node initialized.")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        image_file = self.image_path + 'image_%04d.png' % self.image_count
        cv2.imwrite(image_file, cv_image)
        rospy.loginfo("Saved image: %s" % image_file)
        self.image_count += 1

if __name__ == '__main__':
    rospy.init_node('image_capture', anonymous=True)
    ic = ImageCapture()
    rospy.loginfo("Image capture node is running. Press Ctrl+C to stop.")
    rospy.spin()

