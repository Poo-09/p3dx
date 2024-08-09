#!/usr/bin/env python3
# aruco_detector.py
import rospy
import cv2
from cv2 import aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import yaml
import os

class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()

        # Load camera calibration data
        calib_data_path = "/home/poonam/catkin_ws/src/camera_calibration/ardrone_bottom.yaml"
        if not os.path.exists(calib_data_path):
            rospy.logerr(f"Calibration file not found: {calib_data_path}")
            rospy.signal_shutdown("Calibration file not found")
            return

        try:
            with open(calib_data_path, "r") as file:
                calib_data = yaml.safe_load(file)
                rospy.loginfo(f"Loaded calibration data from {calib_data_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load calibration file: {e}")
            rospy.signal_shutdown("Failed to load calibration file")
            return

        try:
            self.cam_mat = np.array(calib_data["camera_matrix"]["data"]).reshape((3, 3))
            self.dist_coef = np.array(calib_data["distortion_coefficients"]["data"]).reshape((1, 5))
        except KeyError as e:
            rospy.logerr(f"Calibration data format error: {e}")
            rospy.signal_shutdown("Calibration data format error")
            return

        self.marker_size = 0.08  # Marker size in meters

        # Create the dictionary for ArUco markers
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.param_markers = aruco.DetectorParameters_create()

        rospy.loginfo("Subscribing to image topic")
        self.image_sub = rospy.Subscriber('/robot1/rgbd_camera/rgbd_camera/image_raw', Image, self.image_callback)
        rospy.loginfo("ArucoDetector node initialized")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rospy.loginfo("Received image and converted to OpenCV format")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image: {e}")
            return

        try:
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            marker_corners, marker_IDs, _ = aruco.detectMarkers(
                gray_frame, self.marker_dict, parameters=self.param_markers
            )
            rospy.loginfo("Detected markers")

            if marker_corners:
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                    marker_corners, self.marker_size, self.cam_mat, self.dist_coef
                )
                for ids, corners, i in zip(marker_IDs, marker_corners, range(len(marker_IDs))):
                    cv2.polylines(
                        frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                    )
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = corners[0].ravel()
                    bottom_right = corners[2].ravel()

                    # Calculate the distance
                    distance = np.sqrt(
                        tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                    )

                    # Draw the pose of the marker
                    cv2.drawFrameAxes(frame, self.cam_mat, self.dist_coef, rVec[i], tVec[i], 0.1)

                    cv2.putText(
                        frame,
                        f'id: {ids[0]} Dist: {round(distance, 2)}',
                        tuple(top_right),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.3,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        frame,
                        f'x: {round(tVec[i][0][0], 1)} y: {round(tVec[i][0][1], 1)} z: {round(tVec[i][0][2], 1)}',
                        tuple(bottom_right),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.0,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )

                    # Log pose information
                    rospy.loginfo(f"Marker ID: {ids[0]}")
                    rospy.loginfo(f"Translation Vector (tVec): {tVec[i]}")
                    rospy.loginfo(f"Rotation Vector (rVec): {rVec[i]}")

            cv2.imshow('frame', frame)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error during image processing: {e}")

if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)
    try:
        ad = ArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
