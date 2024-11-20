#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


TOPIC_CONTROL = '/quad/cmd_vel'
TOPIC_IMAGE_FEED = '/quad/downward_cam/down_camera/image'

SCAN_HEIGHT = 100
LINEAR_SPEED = 0.2  # Base linear speed
ANGULAR_SPEED = 0.5  # Base angular speed

# PID Constants
PID_KP_LINEAR = 0.5
PID_KD_LINEAR = 0.2
PID_KP_ANGULAR = 0.005
PID_KD_ANGULAR = 0.002

# Canny Edge Detection Thresholds
CANNY_LOW_THRESHOLD = 50
CANNY_HIGH_THRESHOLD = 150

class RoadDriving:
    def __init__(self):
        rospy.init_node('road_driving', anonymous=True)

        self.vel_pub = rospy.Publisher(TOPIC_CONTROL, Twist, queue_size=1)
        rospy.Subscriber('/state', String, self.state_callback)
        rospy.Subscriber('/clue_count', Int8, self.clue_count_callback)
        rospy.Subscriber(TOPIC_IMAGE_FEED, Image, self.image_callback)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(60)

        self.state = ""
        self.clue_count = 0
        self.image = None
        self.previous_angular_error = 0
        self.previous_linear_error = 0

    def state_callback(self, state):
        self.state = state.data

    def clue_count_callback(self, count):
        self.clue_count = count.data

    def image_callback(self, image):
        try:
            self.image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.loginfo(e)

    def update_velocity(self, forward, sideways, up, yaw):
        vel_msg = Twist()
        vel_msg.linear.x = forward
        vel_msg.linear.y = sideways
        vel_msg.linear.z = up
        vel_msg.angular.z = yaw
        self.vel_pub.publish(vel_msg)


    def get_road_center(self, frame):
        """
        Improved logic to detect parallel lines by scanning multiple rows in a vertical band.
        Returns the x-coordinate of the chosen line and the frame center.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape

        # Define the vertical band (range of rows) to search for edges
        target_row = height - SCAN_HEIGHT  # Central row of the band
        band_height = 60  # Height of the vertical band
        row_start = max(0, target_row - band_height // 2)  # Top of the band
        row_end = min(height - 1, target_row + band_height // 2)  # Bottom of the band

        # Apply edge detection to the vertical band
        roi = gray[row_start:row_end, :]
        edges = cv2.Canny(roi, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD)

        # Debug: Visualize the detected edges in the vertical band
        debug_image = np.zeros_like(gray)
        debug_image[row_start:row_end, :] = edges
        cv2.imshow("Edges in Vertical Band", debug_image)
        cv2.waitKey(1)

        # Aggregate edge x-coordinates across all rows in the band
        edge_points = np.where(edges > 0)
        if len(edge_points[1]) == 0:
            rospy.loginfo("No edges detected in vertical band")
            return -1, width // 2

        # Extract unique x-coordinates of edge points
        x_coords = sorted(np.unique(edge_points[1]))

        # Look for pairs of edges that represent parallel lines
        parallel_pairs = []
        for i, x1 in enumerate(x_coords):
            for j, x2 in enumerate(x_coords):
                if i != j:
                    # Check if lines are approximately parallel and within valid spacing
                    if 5 < abs(x1 - x2) < 50:  # Adjust thresholds as needed
                        parallel_pairs.append((x1, x2))

        if not parallel_pairs:
            rospy.loginfo("No valid parallel lines detected")
            return -1, width // 2

        # Select the pair of lines closest to the expected lane region
        expected_lane_center = width // 4  # Adjust for left or right lane following
        best_pair = min(parallel_pairs, key=lambda pair: abs(pair[0] - expected_lane_center))

        # Choose one line to follow: left or right
        # To follow the left line:
        target_edge = min(best_pair)
        # Uncomment to follow the right line instead:
        # target_edge = max(best_pair)

        # Debug: Draw the selected lines
        contour_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.line(contour_image, (best_pair[0], row_start), (best_pair[0], row_end), (255, 0, 0), 2)
        cv2.line(contour_image, (best_pair[1], row_start), (best_pair[1], row_end), (0, 255, 0), 2)
        cv2.circle(contour_image, (target_edge, (row_start + row_end) // 2), 5, (0, 255, 255), -1)
        cv2.imshow("Selected Lines in Vertical Band", contour_image)
        cv2.waitKey(1)

        rospy.loginfo(f"Selected Lines: Left = {best_pair[0]}, Right = {best_pair[1]}, Target Line = {target_edge}")

        return target_edge, width // 2



    def pid_control(self, error, previous_error, kp, kd):
        """
        Computes PID control output using proportional and derivative terms.
        """
        derivative = error - previous_error
        control = kp * error + kd * derivative
        return control, error

    def line_follow(self):
        if self.image is None:
            return

        road_center, frame_center = self.get_road_center(self.image)

        if road_center == -1:  # No road detected
            #self.update_velocity(0, 0, 0, 0.2)  # Rotate in place
            return

        # Compute errors
        angular_error = frame_center - road_center
        linear_error = abs(angular_error)

        # Apply PID for angular velocity
        angular_velocity, self.previous_angular_error = self.pid_control(
            angular_error, self.previous_angular_error, PID_KP_ANGULAR, PID_KD_ANGULAR
        )

        # Apply PID for linear velocity
        linear_velocity, self.previous_linear_error = self.pid_control(
            linear_error, self.previous_linear_error, PID_KP_LINEAR, PID_KD_LINEAR
        )
        linear_velocity = LINEAR_SPEED * (1 - min(1.0, linear_velocity / frame_center))

        self.update_velocity(linear_velocity, 0, 0, angular_velocity)

        # Visualization for debugging
        vis_image = self.image.copy()
        height = vis_image.shape[0]
        cv2.circle(vis_image, (road_center, height - SCAN_HEIGHT), 15, (0, 255, 0), -1)
        cv2.imshow("Processed Image", vis_image)
        cv2.waitKey(1)

    def operate(self):
        """Main loop to check state and drive accordingly."""
        while not rospy.is_shutdown():
            if self.state == "STARTUP":
                self.update_velocity(0, 0, 0, 0)
                

            elif self.state == "DRIVING":
                if self.clue_count == 0:
                    while self.image is None:  # Wait for an image to be received
                        pass
                    self.line_follow()
                elif self.clue_count == 1:
                    self.line_follow()

            elif self.state == "STOP":
                self.update_velocity(0, 0, 0, 0)

            rospy.sleep(0.1)  # Sleep for a short time to avoid high CPU usage

    def start(self):
        self.operate()


if __name__ == '__main__':
    road_driving = RoadDriving()
    rospy.loginfo("Road Driving Node Initialized")
    road_driving.start()
