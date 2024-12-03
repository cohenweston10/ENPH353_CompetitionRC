#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import sys
import threading 


THRESHOLD = 50 #value is based off of lab 2

TOPIC_CONTROL = '/quad/cmd_vel'

LINEAR_SPEED = 0.2
ANGULAR_SPEED = 1.5


class Driving:
    def __init__(self):
        # Initialize the node
        rospy.init_node('driving', anonymous=True)

        self.vel_pub = rospy.Publisher(TOPIC_CONTROL, Twist, queue_size=1)
        self.ready_pub = rospy.Publisher('/ready', String, queue_size=10)

        self.lock = threading.Lock()

        # Subscribe t        self.leaving_prev_clue = Trueo the State output
        rospy.Subscriber('/state', String, self.state_callback)

        # Subscribe to the Clue Count output
        rospy.Subscriber('/clue_count', Int8, self.clue_count_callback)

        # Subscribe to the OCR output
        # rospy.Subscriber('/ocr/processed_strings', String, self.OCRcallback)

 
        self.bridge = CvBridge()
        self.rate = rospy.Rate(30)

        self.localize_duration = 4

        self.state = ""
        self.clue_count = 0
        self.leaving_prev_clue = True
        self.movement_complete = False

        # init images
        self.front_cam_image = None
        self.right_cam_image = None
        self.left_cam_image = None
        self.back_cam_image = None


        # Subscribe to the camera topics
        rospy.Subscriber('/quad/front_cam/camera/image', Image, self.front_image_callback)
        rospy.Subscriber('/quad/right_cam/right_camera/image', Image, self.right_image_callback)
        rospy.Subscriber('/quad/left_cam/left_camera/image', Image, self.left_image_callback)
        rospy.Subscriber('/quad/back_cam/back_camera/image', Image, self.back_image_callback)

        # keypoints and descriptors for ref image
        self.sift = cv2.SIFT_create()
        self.reference_image = cv2.imread("/home/fizzer/ros_ws/src/RC_controller/nodes/clue_banner_ref.png", cv2.IMREAD_GRAYSCALE)
        self.ref_keypoints, self.ref_descriptors = self.sift.detectAndCompute(self.reference_image, None)

        # timing variables
        self.start_time = rospy.Time.now()

        #Driving flags
        self.gts1 = False
        self.gts2 = False
        self.gts3 = False
        self.gts4 = False
        self.gts5 = False
        self.gts6 = False
        self.gts7 = False
        self.gtt = False
        self.gts8 = False
        
    def state_callback(self, state):
        with self.lock:
            self.state = state.data

    def clue_count_callback(self, count):
        with self.lock:
            self.clue_count = count.data
            self.movement_complete = False
            self.ready_pub.publish("NOTREADY")
            rospy.loginfo("CLUE COUNTER INCREMENTED\n\n\n")


    def OCRcallback(self, data):
        if time.time() - self.last_clue > 15:
            self.clue_searching = True
            self.leaving_prev_clue = False
            rospy.loginfo(f"Searching for clue {self.clue_count + 1}, time since last clue is {time.time() - self.last_clue}")

    def front_image_callback(self, image):
        try:
            # Convert the ROS image message to OpenCV format with the correct encoding
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            # Ensure the image is in the correct depth
            self.front_cam_image = cv_image.astype(np.uint8)        
        except CvBridgeError as e:
            rospy.loginfo(e)
            return

    def right_image_callback(self, image):
        try:
            # Convert the ROS image message to OpenCV format with the correct encoding
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            # Ensure the image is in the correct depth
            self.right_cam_image = cv_image.astype(np.uint8)        
        except CvBridgeError as e:
            rospy.loginfo(e)
            return

    def left_image_callback(self, image):
        try:
            # Convert the ROS image message to OpenCV format with the correct encoding
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            # Ensure the image is in the correct depth
            self.left_cam_image = cv_image.astype(np.uint8)        
        except CvBridgeError as e:
            rospy.loginfo(e)
            return

    def back_image_callback(self, image):
        try:
            # Convert the ROS image message to OpenCV format with the correct encoding
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            # Ensure the image is in the correct depth
            self.back_cam_image = cv_image.astype(np.uint8)        
        except CvBridgeError as e:
            rospy.loginfo(e)
            return

    def update_velocity(self, vx, vy, vz, vaz):
        vel_msg = Twist()
        vel_msg.linear.x = vx
        vel_msg.linear.y = vy
        vel_msg.linear.z = vz
        vel_msg.angular.z = vaz
        # rospy.loginfo("Publishing New Velocity")
        self.vel_pub.publish(vel_msg)


    def move_for_duration(self, vx, vy, vz, vaz, duration, ramp_time=1.0):
        """
        Moves the quadrotor with smooth acceleration and deceleration.

        Parameters:
            vx (float): Target linear velocity in the x direction.
            vy (float): Target linear velocity in the y direction.
            vz (float): Target linear velocity in the z direction.
            vaz (float): Target angular velocity around the z axis.
            duration (float): Total time to move in seconds.
            ramp_time (float): Time for acceleration and deceleration in seconds.
        """
        vel_msg = Twist()

        # Ensure ramp_time is not longer than half the duration
        ramp_time = min(ramp_time, duration / 2.0)

        start_time = rospy.Time.now()
        elapsed_time = 0

        while elapsed_time < duration and not rospy.is_shutdown():
            elapsed_time = (rospy.Time.now() - start_time).to_sec()

            if elapsed_time < ramp_time:  # Acceleration phase
                factor = elapsed_time / ramp_time
            elif elapsed_time > (duration - ramp_time):  # Deceleration phase
                factor = (duration - elapsed_time) / ramp_time
            else:  # Cruise phase
                factor = 1.0

            # Scale velocities based on the current phase
            vel_msg.linear.x = factor * vx
            vel_msg.linear.y = factor * vy
            vel_msg.linear.z = factor * vz
            vel_msg.angular.z = factor * vaz

            # Publish the velocity message
            self.vel_pub.publish(vel_msg)
            self.rate.sleep()

        # Stop the movement after the duration
        self.update_velocity(0, 0, 0, 0)
        rospy.loginfo(f"Movement completed with smooth acceleration and deceleration for {duration} seconds.")


    def startup(self):
        self.move_for_duration(0,0,0.20,0,1.5)

    def startupr(self):
        self.move_for_duration(0,0,-0.20,0,1.5)


    def go_sign1(self):
        self.move_for_duration(0.42,0.25,0,0,2)
        self.move_for_duration(0,0,0,0,1)
        self.gts1 = True

    def go_sign1r(self):
        self.move_for_duration(-0.42,-0.25,0,0,2)

    def go_sign2(self):
        self.move_for_duration(0,-0.47,0,0,2.25)
        self.move_for_duration(1.3,0,0,0,3.3)
        self.gts2 = True

    def go_sign2r(self):
            self.move_for_duration(-1.3,0,0,0,3.3)
            self.move_for_duration(0,0.45,0,0,2.25)

    def go_sign3(self):
        self.move_for_duration(0,-0.5,0,0,2.85)
        self.move_for_duration(0.5,0,0,0,1.95)
        self.gts3 = True

    def go_sign3r(self):
        self.move_for_duration(-0.5,0,0,0,1.95)
        self.move_for_duration(0,0.5,0,0,2.85)


    def go_sign4(self):
        self.move_for_duration(-0.6,0,0,0,2.49)
        self.move_for_duration(0,-1.8,0,0,2.86)
        self.gts4 = True

    def go_sign4r(self):
        self.move_for_duration(0,1.8,0,0,2.86)
        self.move_for_duration(0.6,0,0,0,2.49)

    def go_sign5(self):
        self.move_for_duration(0,0,0.5,0,0.4)
        self.move_for_duration(-1.45,0,0,0,2.71)
        self.move_for_duration(0,0,0,0,0.7)
        self.move_for_duration(0,0,-0.5,0,0.4)
        self.gts5 = True

    def go_sign5r(self):
        self.move_for_duration(0,0,0.5,0,0.4)
        self.move_for_duration(1.45,0,0,0,2.71)
        self.move_for_duration(0,0,0,0,0.7)
        self.move_for_duration(0,0,-0.5,0,0.4)

    def go_sign6(self):
        self.move_for_duration(0,-1.45,0,0,3.65)
        self.move_for_duration(0,-0.51,0,0,0.650)
        self.gts6 = True

    def go_sign6r(self):
        self.move_for_duration(0,1.45,0,0,3.65)
        self.move_for_duration(0,0.51,0,0,0.650)


    def go_sign7(self):
        self.move_for_duration(0,0,0.5,0,1)
        self.move_for_duration(0,-0.5,0,0,2.728)
        self.move_for_duration(1.8,0,0,0,3.05)
        self.move_for_duration(0,0,0,0,0.5)
        self.move_for_duration(0,0,-0.5,0,1)
        self.gts7 = True

    def go_sign7r(self):
        self.move_for_duration(0,0,0.5,0,1)
        self.move_for_duration(-1.8,0,0,0,3.05)
        self.move_for_duration(0,0,0,0,0.5)
        self.move_for_duration(0,0.5,0,0,2.728)
        self.move_for_duration(0,0,-0.5,0,1)

    def go_tunnel(self):
        self.move_for_duration(0,0.5,0.25,0,2.5)
        self.gtt = True

    def go_tunnelr(self):
        self.move_for_duration(0,-0.5,-0.25,0,2.5)

    def go_sign8(self):
        self.move_for_duration(-1.13,0,2.05,0,1.4)
        self.move_for_duration(0,0,0,0,0.55)
        self.move_for_duration(0,1.5,0,0,2.5)
        self.move_for_duration(0,0.005,0,0,0.4)
        self.gts8 = True

    def go_sign8r(self):
        self.move_for_duration(0,-0.005,0,0,0.4)
        self.move_for_duration(0,-1.5,0,0,2.55)
        self.move_for_duration(0,0,0,0,0.5)
        self.move_for_duration(1.13,0,-2.05,0,1.4)



    def operate(self):
        """Main loop to check state and drive accordingly."""

        while not rospy.is_shutdown():

            with self.lock:
                # rospy.loginfo(f"Current clue count is {self.clue_count}")
                current_state = self.state
                current_clue_count = self.clue_count
                current_movement_complete = self.movement_complete
                current_start_time = self.start_time
                current_localize_duration = self.localize_duration
                current_front_cam_image = self.front_cam_image
                current_right_cam_image = self.right_cam_image
                current_left_cam_image = self.left_cam_image
                current_back_cam_image = self.back_cam_image


            if self.state == "STARTUP":
                self.startup()
                rospy.loginfo("Starting up...")

            elif self.state == "DRIVING":
                if current_clue_count == 0:
                    if current_movement_complete: # Post operation
                        self.ready_pub.publish("READY") # do nothing
                    elif not self.gts1: # Regular operation
                        self.go_sign1()
                        with self.lock:
                            self.movement_complete = True
                            self.start_time = rospy.Time.now()


                elif current_clue_count == 1:
                    if current_movement_complete and (rospy.Time.now() - current_start_time).to_sec() < current_localize_duration: # Post operation
                        self.localize(self.front_cam_image, "FRONT")
                    elif current_movement_complete:
                        self.ready_pub.publish("READY")
                    elif not self.gts2: # Regular operation
                        self.go_sign2()
                        with self.lock:
                            self.movement_complete = True
                            self.start_time = rospy.Time.now()
                    else:
                        rospy.loginfo("stuck :(")


                elif current_clue_count == 2:
                    if current_movement_complete and (rospy.Time.now() - current_start_time).to_sec() < current_localize_duration: # Post operation
                        self.localize(self.right_cam_image, "RIGHT")
                    elif current_movement_complete:
                        self.ready_pub.publish("READY")
                    elif not self.gts3: # Regular operation
                        self.go_sign3()
                        with self.lock:
                            self.movement_complete = True
                            self.start_time = rospy.Time.now()


                elif current_clue_count == 3:
                    if current_movement_complete and (rospy.Time.now() - current_start_time).to_sec() < current_localize_duration: # Post operation
                        self.localize(self.back_cam_image, "BACK")
                    elif current_movement_complete:
                        self.ready_pub.publish("READY")
                    elif not self.gts4: # Regular operation
                        self.go_sign4()
                        with self.lock:
                            self.movement_complete = True
                            self.start_time = rospy.Time.now()


                elif current_clue_count == 4:
                    if current_movement_complete and (rospy.Time.now() - current_start_time).to_sec() < 2: # Post operation
                        self.localize(self.front_cam_image, "FRONT")
                    elif current_movement_complete:
                        self.ready_pub.publish("READY")
                    elif not self.gts5: # Regular operation
                        self.go_sign5()
                        with self.lock:
                            self.movement_complete = True
                            self.start_time = rospy.Time.now()


                elif current_clue_count == 5:
                    if current_movement_complete and (rospy.Time.now() - current_start_time).to_sec() < 3: # Post operation
                        self.localize(self.right_cam_image, "RIGHT")
                    elif current_movement_complete:
                        self.ready_pub.publish("READY")
                    elif not self.gts6: # Regular operation
                        self.go_sign6()
                        with self.lock:
                            self.movement_complete = True
                            self.start_time = rospy.Time.now()


                elif current_clue_count == 6:
                    if current_movement_complete and (rospy.Time.now() - current_start_time).to_sec() < current_localize_duration: # Post operation
                        self.localize(self.left_cam_image, "LEFT")
                    elif current_movement_complete:
                        self.ready_pub.publish("READY")
                    elif not self.gts7: # Regular operation
                        self.go_sign7()
                        with self.lock:
                            self.movement_complete = True
                            self.start_time = rospy.Time.now()


                elif current_clue_count == 7:
                    if current_movement_complete and (rospy.Time.now() - current_start_time).to_sec() < current_localize_duration: # Post operation
                        self.localize(self.left_cam_image, "LEFT")
                    elif current_movement_complete:
                        self.ready_pub.publish("READY")
                    elif not self.gts8: # Regular operation
                        self.go_tunnel()
                        self.go_sign8()
                        with self.lock:
                            self.movement_complete = True
                            self.start_time = rospy.Time.now()

            elif current_state == "STOP":
                self.update_velocity(0, 0, 0, 0)
                with self.lock:
                    self.leaving_prev_clue = True

            rospy.sleep(0.1)  # Sleep for a short time to avoid high CPU usage


    def localize(self, cam, direction):
        cv_image = cam

        if cv_image is None or cv_image.size == 0:
            rospy.logwarn("No image data received.")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect keypoints and descriptors in the current frame
        kp, des = self.sift.detectAndCompute(gray, None)

        if des is None or len(des) == 0:
            rospy.logwarn("No descriptors found in the current frame.")
            return

        if self.ref_descriptors is None or len(self.ref_descriptors) == 0:
            rospy.logwarn("No reference descriptors available.")
            return

        # Set up feature matcher (FLANN-based)
        index_params = dict(algorithm=0, trees=5)
        search_params = dict()
        flann = cv2.FlannBasedMatcher(index_params, search_params)

        # Match descriptors
        matches = flann.knnMatch(des, self.ref_descriptors, k=2)

        # Filter good matches using Lowe's ratio test
        good_matches = []
        for m, n in matches:
            if m.distance < 0.6 * n.distance and m.queryIdx < len(kp) and m.trainIdx < len(self.ref_keypoints):
                good_matches.append(m)

        # Validate that both keypoints and matches exist
        if len(self.ref_keypoints) == 0 or len(kp) == 0:
            rospy.logwarn("No keypoints detected in reference or current frame.")
            return

        if len(good_matches) == 0:
            rospy.logwarn("No good matches found.")
            return

        # Draw matches with valid indices
        try:
            # match_img = cv2.drawMatches(self.reference_image, self.ref_keypoints, gray, kp, good_matches[:10], None)
            # cv2.imshow("Matches", match_img)
            # cv2.waitKey(1)
            x = 2
        except Exception as e:
            rospy.logerr(f"Error while drawing matches: {e}")

        # Estimate pose if enough matches are found
        if len(good_matches) > 3:
            src_pts = np.float32([self.ref_keypoints[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)

            # Compute homography
            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            if H is not None:
                # Find the center of the matched reference image in the current frame
                h, w = self.reference_image.shape
                ref_center = np.array([[w / 2, h / 2]], dtype=np.float32).reshape(-1, 1, 2)
                ref_center_transformed = cv2.perspectiveTransform(ref_center, H)

                # Current frame center
                frame_center_x = gray.shape[1] / 2
                frame_center_y = gray.shape[0] / 2

                # Define the target position slightly left of the center
                target_x = frame_center_x
                target_y = frame_center_y

                # Difference between transformed reference center and the target position
                if direction == "FRONT":
                    diff_x = ref_center_transformed[0, 0, 0] - target_x
                    rospy.loginfo(f"Center is at {ref_center_transformed[0, 0, 0]}, target is {target_x}, difference is {diff_x}")
                    self.update_velocity(0, -0.0005 * diff_x, 0, 0)
                elif direction == "BACK":
                    diff_x = -1 * (ref_center_transformed[0, 0, 0] - target_x)
                    self.update_velocity(0, -0.0005 * diff_x, 0, 0)
                elif direction == "RIGHT":
                    diff_x = ref_center_transformed[0, 0, 0] - target_x
                    self.update_velocity(-0.0005 * diff_x, 0, 0, 0)
                elif direction == "LEFT":
                    diff_x = -1 * (ref_center_transformed[0, 0, 0] - target_x)
                    self.update_velocity(-0.0005 * diff_x, 0, 0, 0)

                rospy.loginfo(f"Localization velocitiy published: {-0.0005 * diff_x}")
            else:
                # rospy.logwarn("Homography matrix could not be computed.")
                x = 2
        else:
            self.update_velocity(0,0,0,0)

    def start(self):
        # Start the ROS loop
        self.operate()

    #For testing purposes
    def listen_for_commands(self):
        rospy.loginfo("Listening for commands. Type 'go_sign1' or 'go_sign2'. Type 'exit' to quit.")
        while not rospy.is_shutdown():
            try:
                command = input("Enter command: ").strip()
                if command == 'go_startup':
                    self.startup()
                elif command == 'go_sign1':
                    self.go_sign1()
                elif command == 'go_startupr':
                    self.startupr()
                elif command == 'go_sign1r':
                    self.go_sign1r()
                elif command == 'go_sign2':
                    self.go_sign2()
                elif command == 'go_sign2r':
                    self.go_sign2r()
                elif command == 'go_sign3':
                    self.go_sign3()
                elif command == 'go_sign3r':
                    self.go_sign3r()
                elif command == 'go_sign4':
                    self.go_sign4()
                elif command == 'go_sign4r':
                    self.go_sign4r()
                elif command == 'go_sign5':
                    self.go_sign5()
                elif command == 'go_sign5r':
                    self.go_sign5r()
                elif command == 'go_sign6':
                    self.go_sign6()
                elif command == 'go_sign6r':
                    self.go_sign6r()
                elif command == 'go_sign7':
                    self.go_sign7()
                elif command == 'go_sign7r':
                    self.go_sign7r()    
                elif command == 'go_tunnel':
                    self.go_tunnel()
                elif command == 'go_tunnelr':
                    self.go_tunnelr()   
                elif command == 'go_sign8':
                    self.go_sign8()
                elif command == 'go_sign8r':
                    self.go_sign8r()    
                elif command == 'exit':
                    rospy.loginfo("Exiting command listener.")
                    break
                else:
                    rospy.logwarn(f"Unknown command: {command}")
            except KeyboardInterrupt:
                rospy.loginfo("Shutting down command listener.")
                break

if __name__ == '__main__':
    # Create an instance of the Driving class
    driving = Driving()
    rospy.loginfo("Road Driving Node Initialized")
    #driving.listen_for_commands()
    driving.start()