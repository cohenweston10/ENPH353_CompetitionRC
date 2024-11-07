#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()
sift = cv2.SIFT_create()

def read_sign(data):
    # Convert the ROS Image message to OpenCV format
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

    # Convert image to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Detect SIFT features (keypoints) and descriptors for camera image
    keypoints, descriptors = sift.detectAndCompute(gray, None)

    # Load reference image and process it
    ref_image = cv2.imread("Sign_Example3.png")
    ref_gray = cv2.cvtColor(ref_image, cv2.COLOR_BGR2GRAY)
    kp_ref, desc_ref = sift.detectAndCompute(ref_gray, None)

    # Set up feature matcher (FLANN-based)
    index_params = dict(algorithm=0, trees=5)
    search_params = dict()
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    # Match descriptors
    matches = flann.knnMatch(descriptors, desc_ref, k=2)

    # Filter good matches using Lowe's ratio test and ensure indices are valid
    good_matches = []
    for m, n in matches:
        if m.distance < 0.6 * n.distance and m.queryIdx < len(keypoints) and m.trainIdx < len(kp_ref):
            good_matches.append(m)

    # If enough matches are found, compute homography
    if len(good_matches) > 3:
        # Get matched keypoints in both images
        query_pts = np.float32([kp_ref[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        train_pts = np.float32([keypoints[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # Compute homography
        matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
        if matrix is not None:
            # Draw perspective transform of the reference image on the camera frame
            h, w = ref_gray.shape
            pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
            dst = cv2.perspectiveTransform(pts, matrix)

            # Draw the transformed reference image border
            homography_image = cv2.polylines(cv_image, [np.int32(dst)], True, (255, 0, 0), 3)
            cv2.imshow("Camera - Homography Detection", homography_image)
            cv2.waitKey(1)

    kp_image = cv2.drawKeypoints(cv_image, keypoints, cv_image)
    cv2.imshow("Camera - Keypoints", kp_image)
    cv2.waitKey(1)

def main():
    # Initialize the ROS node
    rospy.init_node('sign_reader', anonymous=True)

    # Subscribe to the camera topic
    rospy.Subscriber('/quad/front_cam/camera/image', Image, read_sign)

    rospy.spin()

if __name__ == '__main__':
    main()