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


THRESHOLD = 50 #value is based off of lab 2

TOPIC_CONTROL = '/quad/cmd_vel'

LINEAR_SPEED = 0.2
ANGULAR_SPEED = 1.5


class Driving:
    def __init__(self):
        # Initialize the node
        rospy.init_node('driving', anonymous=True)

        self.vel_pub = rospy.Publisher(TOPIC_CONTROL, Twist, queue_size=1)

        # Subscribe t        self.approaching_clue = Trueo the State output
        rospy.Subscriber('/state', String, self.state_callback)

        # Subscribe to the Clue Count output
        rospy.Subscriber('/clue_count', Int8, self.clue_count_callback)

        # Subscribe to the OCR output
        rospy.Subscriber('/ocr/processed_strings', String, self.OCRcallback)


        self.bridge = CvBridge()
        self.rate = rospy.Rate(60)


        self.state = ""
        self.clue_count = 0
        self.approaching_clue = True
        self.clue_searching = False
        self.last_clue = -5 # time since reading last clue
        
    def state_callback(self, state):
        self.state = state.data

    def clue_count_callback(self, count):
        self.clue_count = count.data
        self.approaching_clue = True
        self.clue_searching = False
        self.last_clue = time.time()
        rospy.loginfo("CLUE COUNTER INCREMENTED\n\n\n")

    def OCRcallback(self, data):
        if time.time() - self.last_clue > 15:
            self.clue_searching = True
            self.approaching_clue = False
            rospy.loginfo(f"Searching for clue {self.clue_count + 1}, time since last clue is {time.time() - self.last_clue}")

    def update_velocity(self, vx, vy, vz, vaz):
        vel_msg = Twist()
        vel_msg.linear.x = vx
        vel_msg.linear.y = vy
        vel_msg.linear.z = vz
        vel_msg.angular.z = vaz
        # rospy.loginfo("Publishing New Velocity")
        self.vel_pub.publish(vel_msg)

    def operate(self):
        """Main loop to check state and drive accordingly."""
        while not rospy.is_shutdown():
            if self.state == "STARTUP":
                self.update_velocity(0,0,0,0)
                rospy.loginfo("Starting up...")

            #elif self.state == "DRIVING":
            else:
                self.update_velocity(0,0,0.2,0) # put thisin startup later
                rospy.sleep(1)
                self.update_velocity(0,0,0,0)

                if self.clue_count == 0:
                    if self.approaching_clue: # Pre operation
                        rospy.loginfo("Approaching clue 1")
                        rospy.sleep(1)

                    if self.clue_searching: # Post operation
                        rospy.loginfo("Searching for clue 1")
                        rospy.sleep(1)
                        continue

                    else: # Regular operation
                        continue


                elif self.clue_count == 1:

                    if self.approaching_clue: # Pre operation
                        rospy.loginfo("Approaching clue 2")
                        rospy.sleep(1)

                    if self.clue_searching: # Post operation
                        rospy.loginfo("Searching for clue 2")
                        rospy.sleep(1)
                        continue

                    else: # Regular operation
                        continue

                elif self.clue_count == 2:
                    if self.approaching_clue: # Pre operation
                        rospy.loginfo("Approaching clue 3")
                        rospy.sleep(1)

                    if self.clue_searching: # Post operation
                        rospy.loginfo("Searching for clue 3")
                        rospy.sleep(1)
                        continue

                    else: # Regular operation
                        continue

                elif self.clue_count == 3:
                    return

                elif self.clue_count == 4:
                    return

                elif self.clue_count == 5:
                    return

                elif self.clue_count == 6:
                    return

                elif self.clue_count == 7:
                    return

                elif self.clue_count == 8:
                    return

            # elif self.state == "STOP":
            #     self.update_velocity(0, 0, 0, 0)
            #     self.approaching_clue = True

            rospy.sleep(0.1)  # Sleep for a short time to avoid high CPU usage



    def start(self):
        # Start the ROS loop
        self.operate()

if __name__ == '__main__':
    # Create an instance of the Driving class
    driving = Driving()
    rospy.loginfo("Road Driving Node Initialized")
    driving.start()