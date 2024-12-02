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

        # Subscribe t        self.leaving_prev_clue = Trueo the State output
        rospy.Subscriber('/state', String, self.state_callback)

        # Subscribe to the Clue Count output
        rospy.Subscriber('/clue_count', Int8, self.clue_count_callback)

        # Subscribe to the OCR output
        rospy.Subscriber('/ocr/processed_strings', String, self.OCRcallback)


        self.bridge = CvBridge()
        self.rate = rospy.Rate(30)


        self.state = ""
        self.clue_count = 0
        self.leaving_prev_clue = True
        self.clue_searching = False
        self.last_clue = -5 # time since reading last clue
        
    def state_callback(self, state):
        self.state = state.data

    def clue_count_callback(self, count):
        self.clue_count = count.data
        self.leaving_prev_clue = True
        self.clue_searching = False
        self.last_clue = time.time()
        rospy.loginfo("CLUE COUNTER INCREMENTED\n\n\n")

    def OCRcallback(self, data):
        if time.time() - self.last_clue > 15:
            self.clue_searching = True
            self.leaving_prev_clue = False
            rospy.loginfo(f"Searching for clue {self.clue_count + 1}, time since last clue is {time.time() - self.last_clue}")

    def update_velocity(self, vx, vy, vz, vaz):
        vel_msg = Twist()
        vel_msg.linear.x = vx
        vel_msg.linear.y = vy
        vel_msg.linear.z = vz
        vel_msg.angular.z = vaz
        # rospy.loginfo("Publishing New Velocity")
        self.vel_pub.publish(vel_msg)


    def move_for_duration(self, vx, vy, vz, vaz, duration):
        """
        Moves the quadrotor in the specified direction for a given duration and then stops.

        Parameters:
            vx (float): Linear velocity in the x direction.
            vy (float): Linear velocity in the y direction.
            vz (float): Linear velocity in the z direction.
            vaz (float): Angular velocity around the z axis.
            duration (float): Time to move in seconds.
        """
        vel_msg = Twist()
        vel_msg.linear.x = vx
        vel_msg.linear.y = vy
        vel_msg.linear.z = vz
        vel_msg.angular.z = vaz

        # Publish the velocity command for the specified duration
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(duration):
            self.vel_pub.publish(vel_msg)
            self.rate.sleep()

        # Stop the movement after the duration
        self.update_velocity(0, 0, 0, 0)
        rospy.loginfo(f"Movement in direction ({vx}, {vy}, {vz}, {vaz}) completed for {duration} seconds.")



    def operate(self):
        """Main loop to check state and drive accordingly."""
        while not rospy.is_shutdown():
            if self.state == "STARTUP":
                self.update_velocity(0,0,0.11,0)
                #rospy.loginfo("Starting up...")

            elif self.state == "DRIVING":
                if self.clue_count == 0:
                    if self.leaving_prev_clue: # Pre operation
                        self.move_for_duration(0.12,0.07,0.06,0,3)
                        self.leaving_prev_clue = False

                    if self.clue_searching: # Post operation
                        #rospy.loginfo("Searching for clue 1")
                        self.update_velocity(0,0,0,0)
                        #rospy.sleep(1)
                        continue

                    else: # Regular operation
                        self.update_velocity(0.01,0,0,0)


                elif self.clue_count == 1:

                    if self.leaving_prev_clue: # Pre operation
                        self.move_for_duration(0,-0.3,0,0,1.3)
                        self.move_for_duration(2,0,0,0,1)
                        self.leaving_prev_clue = False

                    if self.clue_searching: # Post operation
                        self.update_velocity(-0.05,0,0,0)

                    else: # Regular operation
                        self.update_velocity(0.15,0,0,0)

                elif self.clue_count == 2:
                    if self.leaving_prev_clue: # Pre operation
                        self.update_velocity(0,0,0,0)
                        self.move_for_duration(0.2,0,0,0,2)
                        self.leaving_prev_clue = False

                    if self.clue_searching: # Post operation
                        self.update_velocity(0,0.05,0,0)

                    else: # Regular operation
                        self.update_velocity(0,-0.15,0,0)

                elif self.clue_count == 3:
                    if self.leaving_prev_clue: # Pre operation
                        self.update_velocity(0,0,0,0)
                        self.move_for_duration(-0.2,0,0,0,3.7)
                        self.move_for_duration(0,-3,0,0,1)
                        self.leaving_prev_clue = False

                    if self.clue_searching: # Post operation
                        self.update_velocity(0.05,0.01,0,0)

                    else: # Regular operation
                        self.update_velocity(0,-0.15,0,0)

                elif self.clue_count == 4:
                    if self.leaving_prev_clue: # Pre operation
                        self.update_velocity(0,0,0,0)
                        self.move_for_duration(0,-0.2,0,0,1)
                        self.leaving_prev_clue = False

                    if self.clue_searching: # Post operation
                        self.update_velocity(0.05,0,0,0)

                    else: # Regular operation
                        self.update_velocity(-0.15,0,0,0)

                elif self.clue_count == 5:
                    if self.leaving_prev_clue: # Pre operation
                        self.update_velocity(0,0,0,0)
                        self.move_for_duration(0.2,0,0,0,1)
                        self.leaving_prev_clue = False

                    if self.clue_searching: # Post operation
                        self.update_velocity(0.05,0,0,0)

                    else: # Regular operation
                        self.update_velocity(0,-0.15,0,0)

                elif self.clue_count == 6:
                    if self.leaving_prev_clue: # Pre operation
                        self.update_velocity(0,0,0,0)
                        self.move_for_duration(0,-0.23,0.04,0,3)
                        self.leaving_prev_clue = False
                        clearedHill = False

                    if self.clue_searching: # Post operation
                        self.update_velocity(-0.05,0,0,0)

                    else: # Regular operation
                        if not clearedHill:
                            self.move_for_duration(0.2,0,0,0,12)
                            self.move_for_duration(0.2,0,-0.2,0,0.5)
                            clearedHill = True
                        else:
                            self.update_velocity(0.15,0,0,0)


                elif self.clue_count == 7:
                    self.update_velocity(0,0,0,0)

                elif self.clue_count == 8:
                    return

            # elif self.state == "STOP":
            #     self.update_velocity(0, 0, 0, 0)
            #     self.leaving_prev_clue = True

            rospy.sleep(0.1)  # Sleep for a short time to avoid high CPU usage



    def start(self):
        # Start the ROS loop
        self.operate()

if __name__ == '__main__':
    # Create an instance of the Driving class
    driving = Driving()
    rospy.loginfo("Road Driving Node Initialized")
    driving.start()