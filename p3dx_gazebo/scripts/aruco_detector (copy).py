#!/usr/bin/env python3
# aruco_detector.py
import rospy
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.param_markers = aruco.DetectorParameters_create()
        self.image_sub = rospy.Subscriber('/robot1/rgbd_camera/rgbd_camera/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, _ = aruco.detectMarkers(
            gray_frame, self.marker_dict, parameters=self.param_markers
        )
        if marker_corners:
            rospy.loginfo("ArUco marker detected!")
            for ids, corners in zip(marker_IDs, marker_corners):
                rospy.loginfo(f"Marker ID: {ids[0]}, Corners: {corners}")
                cv2.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                cv2.putText(
                    frame,
                    f'id: {ids[0]}',
                    tuple(top_right),
                    cv2.FONT_HERSHEY_PLAIN,
                    1.3,
                    (200, 100, 0),
                    2,
                    cv2.LINE_AA,
                )
        cv2.imshow('frame', frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)
    ad = ArucoDetector()
    rospy.spin()
    cv2.destroyAllWindows()

