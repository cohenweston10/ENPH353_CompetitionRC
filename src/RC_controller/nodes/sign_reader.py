#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
import os

class SignReader:
    def __init__(self):
        # Initialize the node
        rospy.init_node('sign_reader', anonymous=True)

        # Initialize the CvBridge and SIFT detector
        self.bridge = CvBridge()
        self.sift = cv2.SIFT_create()

        # Initialize the publisher for homography topic
        self.pub = rospy.Publisher('signreader/homography', Image, queue_size=10)

        # Subscribe to the camera topic
        rospy.Subscriber('/quad/front_cam/camera/image', Image, self.image_callback)
        rospy.Subscriber('/quad/left_cam/left_camera/image', Image, self.image_callback)
        rospy.Subscriber('/quad/right_cam/right_camera/image', Image, self.image_callback)
        rospy.Subscriber('/quad/back_cam/back_camera/image', Image, self.image_callback)


        # Initialize a frame counter
        self.frame_counter = 0

    def read_sign(self, cv_image):
        # Convert image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect SIFT features (keypoints) and descriptors for the camera image
        keypoints, descriptors = self.sift.detectAndCompute(gray, None)

        # Load reference image and process it
        ref_image = cv2.imread("/home/fizzer/ros_ws/src/RC_controller/nodes/clue_banner_ref.png")
        ref_gray = cv2.cvtColor(ref_image, cv2.COLOR_BGR2GRAY)
        kp_ref, desc_ref = self.sift.detectAndCompute(ref_gray, None)
        ref_h, ref_w = ref_gray.shape[:2]

        # Set up feature matcher (FLANN-based)
        index_params = dict(algorithm=0, trees=5)
        search_params = dict()
        flann = cv2.FlannBasedMatcher(index_params, search_params)

        # Match descriptors
        matches = flann.knnMatch(descriptors, desc_ref, k=2)

        # Filter good matches using Lowe's ratio test
        good_matches = []
        for m, n in matches:
            if m.distance < 0.6 * n.distance and m.queryIdx < len(keypoints) and m.trainIdx < len(kp_ref):
                good_matches.append(m)

        # Only proceed if enough good matches are found
        if len(good_matches) > 8:
            # Get matched keypoints in both images
            query_pts = np.float32([keypoints[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            train_pts = np.float32([kp_ref[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

            # Compute homography to map camera image sign to reference orientation
            matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)

            # Only proceed if the matrix is valid
            if matrix is not None:
                # Warp the region in the camera image to match the reference image perspective
                rectified_sign = cv2.warpPerspective(gray, matrix, (ref_w, ref_h))

                rospy.loginfo("Sign detected")
                return rectified_sign  # Output rectified image

        rospy.loginfo("Sign not detected")
        return None  # If no rectified image can be computed

    def image_callback(self, data):
        # Convert ROS Image to OpenCV format
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Increment the frame counter
        self.frame_counter += 1

        # Process only every 10th frame
        if self.frame_counter % 10 == 0:
            # Call the read_sign method to process the image
            rectified_sign = self.read_sign(cv_image)
            if rectified_sign is not None:

                # Publish the rectified image
                rectified_img_msg = self.bridge.cv2_to_imgmsg(rectified_sign, encoding="mono8")
                self.pub.publish(rectified_img_msg)


                # uncomment to display transformed sign for debugging purspose 
                # cv2.imshow("Rectified Sign", rectified_sign)
                # cv2.waitKey(1)

    def start(self):
        # Start the ROS loop
        rospy.spin()

if __name__ == '__main__':
    # Create an instance of the SignReader class and start it
    sign_reader = SignReader()
    rospy.loginfo("Sign Reader Node Initialized.")
    sign_reader.start()