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

SCAN_HEIGHT = 400
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
        Refined logic to calculate the road center by focusing on the four relevant edges
        (two outermost and two lane markings).
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape

        # Use the entire screen as ROI
        roi = gray[:, :]

        # Edge Detection
        edges = cv2.Canny(roi, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD)

        # Debug: Display the edges for tuning
        cv2.imshow("Edge Detection", edges)
        cv2.waitKey(1)  # Ensure the window updates properly

        # Find the x-coordinates of all detected edge pixels
        edge_points = np.where(edges > 0)
        if len(edge_points[1]) == 0:
            rospy.loginfo("No edges detected")
            return -1, width // 2

        # Sort x-coordinates of all edge points
        x_coords = sorted(edge_points[1])

        # Cluster edges by proximity
        edge_clusters = []
        current_cluster = [x_coords[0]]

        for x in x_coords[1:]:
            if abs(x - current_cluster[-1]) < 10:  # Threshold for clustering (tune as needed)
                current_cluster.append(x)
            else:
                edge_clusters.append(current_cluster)
                current_cluster = [x]
        edge_clusters.append(current_cluster)

        # Reduce clusters to their mean x-coordinate
        cluster_centers = [int(np.mean(cluster)) for cluster in edge_clusters]

        # Debug: Log clusters and their centers
        rospy.loginfo(f"Detected Edge Clusters: {cluster_centers}")

        # Ensure we have at least 4 edges (discard excess clusters)
        if len(cluster_centers) < 4:
            rospy.loginfo("Not enough edges detected to define the road")
            return -1, width // 2

        # Sort clusters and pick the two outermost edges and the two inner lane markings
        cluster_centers.sort()
        left_outer = cluster_centers[0]
        right_outer = cluster_centers[-1]

        # The two inner lane markings
        left_inner = cluster_centers[1]
        right_inner = cluster_centers[-2]

        # Compute road center as the midpoint between the two inner lane markings
        road_center = (left_inner + right_inner) // 2

        # Debug: Draw detected edges and road center
        contour_image = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
        cv2.line(contour_image, (left_outer, 0), (left_outer, height), (255, 0, 0), 2)
        cv2.line(contour_image, (right_outer, 0), (right_outer, height), (255, 0, 0), 2)
        cv2.line(contour_image, (left_inner, 0), (left_inner, height), (0, 255, 0), 2)
        cv2.line(contour_image, (right_inner, 0), (right_inner, height), (0, 255, 0), 2)
        cv2.circle(contour_image, (road_center, height // 2), 5, (0, 255, 255), -1)
        cv2.imshow("Relevant Edges and Road Center", contour_image)
        cv2.waitKey(1)  # Ensure the window updates properly

        rospy.loginfo(f"Left Outer: {left_outer}, Left Inner: {left_inner}, Right Inner: {right_inner}, Right Outer: {right_outer}")
        rospy.loginfo(f"Road Center: {road_center}, Frame Center: {width // 2}")

        return road_center, width // 2



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

        #self.update_velocity(linear_velocity, 0, 0, angular_velocity)

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
