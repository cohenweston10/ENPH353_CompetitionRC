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

        # Subscribe t        self.switch_pending = Trueo the State output
        rospy.Subscriber('/state', String, self.state_callback)

        # Subscribe to the Clue Count output
        rospy.Subscriber('/clue_count', Int8, self.clue_count_callback)

        # Subscribe to the OCR output
        rospy.Subscriber('/ocr/processed_strings', String, self.OCRcallback)


        self.bridge = CvBridge()
        self.rate = rospy.Rate(60)


        self.state = ""
        self.clue_count = 0
        self.switch_pending = True
        self.clue_searching = False
        self.last_clue = -5
        
    def state_callback(self, state):
        self.state = state.data

    def clue_count_callback(self, count):
        self.clue_count = count.data
        self.switch_pending = True
        self.clue_searching = False
        self.last_clue = time.time()

    def OCRcallback(self, data):
        if time.time() - self.last_clue > 75:
            self.clue_searching = True
            rospy.loginfo(f"Searching for clue {self.clue_count + 1}, time since last clue is {time.time() - self.last_clue}")

    def update_velocity(self, forward, sideways, up, yaw):
        vx = forward * 0.5  # Forward/Backward
        vy = sideways * 0.5  # Left/Right (corrected)
        vz = up * 0.5  # Up/Down
        vaz = yaw  # Rotate left/right

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
                self.update_velocity(0,0,0.5,0)

            elif self.state == "DRIVING":

                if self.clue_count == 0:
                    if self.switch_pending == True: # Pre operation
                        rospy.loginfo("Swapping Left")
                        self.sideSwap("ToLeft")
                        rospy.sleep(1)

                        self.switch_pending = False

                    if self.clue_searching == True: # Post operation
                        self.update_velocity(0.1,0,0,0)

                    else: # Regular operation
                        self.update_velocity(0.2,0,0,0)


                elif self.clue_count == 1:
                    if self.switch_pending == True: # Pre operation
                        rospy.loginfo("Swapping Right")
                        self.sideSwap("ToRight")
                        rospy.sleep(3)

                        self.switch_pending = False
                        
                    if self.clue_searching == True: # Post operation
                        self.update_velocity(0.1,0,0,0)

                    else: # Regular operation
                        self.update_velocity(0.65,0,0,0) 



                elif self.clue_count == 2:
                    if self.switch_pending == True: # Pre operation
                        rospy.loginfo("Doing state 2 pre-operation")
                        self.localize2()

                        self.switch_pending = False

                    if self.clue_searching == True: # Post operation
                        # TODO state 2 post-operation
                        rospy.loginfo("Looking for sign 3")

                    else:
                        # TODO state 2 regular operation
                        rospy.loginfo("In regular state 2 operation")

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

            elif self.state == "STOP":
                self.update_velocity(0, 0, 0, 0)
                self.switch_pending = True

            rospy.sleep(0.1)  # Sleep for a short time to avoid high CPU usage



    def start(self):
        # Start the ROS loop
        self.operate()

if __name__ == '__main__':
    # Create an instance of the Driving class
    driving = Driving()
    rospy.loginfo("Road Driving Node Initialized")
    driving.start()