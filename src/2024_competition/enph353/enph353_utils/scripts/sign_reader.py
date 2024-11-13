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
    ref_image = cv2.imread("clue_banner_ref.png")
    ref_gray = cv2.cvtColor(ref_image, cv2.COLOR_BGR2GRAY)
    kp_ref, desc_ref = sift.detectAndCompute(ref_gray, None)
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
    if len(good_matches) > 20:
        # Get matched keypoints in both images
        query_pts = np.float32([keypoints[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        train_pts = np.float32([kp_ref[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        # Compute homography to map camera image sign to reference orientation
        matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)

        # Only proceed if the matrix is valid
        if matrix is not None:
            # Warp the region in the camera image to match the reference image perspective
            rectified_sign = cv2.warpPerspective(gray, matrix, (ref_w, ref_h))
            
            # Show the rectified sign, aligned to match the reference orientation
            cv2.imshow("Rectified Sign", rectified_sign)
            cv2.waitKey(1)

    # Display the original camera frame for comparison
    cv2.imshow("Camera View", cv_image)
    cv2.waitKey(1)


def main():
    # Initialize the ROS node
    rospy.init_node('sign_reader', anonymous=True)

    # Subscribe to the camera topic
    rospy.Subscriber('/quad/front_cam/camera/image', Image, read_sign)

    rospy.spin()

if __name__ == '__main__':
    main()
