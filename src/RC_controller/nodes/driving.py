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
        # rospy.Subscriber('/ocr/processed_strings', String, self.OCRcallback)


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

    def go_sign1r(self):
        self.move_for_duration(-0.42,-0.25,0,0,2)

    def go_sign2(self):
        self.move_for_duration(0,-0.47,0,0,2)
        self.move_for_duration(1.3,0,0,0,3.3)

    def go_sign2r(self):
            self.move_for_duration(-1.3,0,0,0,3.3)
            self.move_for_duration(0,0.45,0,0,2)

    def go_sign3(self):
        self.move_for_duration(0,-0.5,0,0,2.85)
        self.move_for_duration(0.5,0,0,0,1.95)

    def go_sign3r(self):
        self.move_for_duration(-0.5,0,0,0,1.95)
        self.move_for_duration(0,0.5,0,0,2.85)


    def go_sign4(self):
        self.move_for_duration(-0.6,0,0,0,2.54)
        self.move_for_duration(0,-1.8,0,0,2.86)

    def go_sign4r(self):
        self.move_for_duration(0,1.8,0,0,2.86)
        self.move_for_duration(0.6,0,0,0,2.54)

    def go_sign5(self):
        self.move_for_duration(0,0,0.5,0,0.4)
        self.move_for_duration(-1.45,0,0,0,2.71)
        self.move_for_duration(0,0,0,0,0.7)
        self.move_for_duration(0,0,-0.5,0,0.4)

    def go_sign5r(self):
        self.move_for_duration(0,0,0.5,0,0.4)
        self.move_for_duration(1.45,0,0,0,2.71)
        self.move_for_duration(0,0,0,0,0.7)
        self.move_for_duration(0,0,-0.5,0,0.4)

    def go_sign6(self):
        self.move_for_duration(0,-1.8,0,0,3.22)

    def go_sign6r(self):
        self.move_for_duration(0,1.8,0,0,3.22)

    def go_sign7(self):
        self.move_for_duration(0,0,0.5,0,1)
        self.move_for_duration(0,-0.5,0,0,3.0)
        self.move_for_duration(1.8,0,0,0,3)
        self.move_for_duration(0,0,0,0,0.5)
        self.move_for_duration(0,0,-0.5,0,1)

    def go_sign7r(self):
        self.move_for_duration(0,0,0.5,0,1)
        self.move_for_duration(-1.8,0,0,0,3)
        self.move_for_duration(0,0,0,0,0.5)
        self.move_for_duration(0,0.5,0,0,3.0)
        self.move_for_duration(0,0,-0.5,0,1)

    def go_tunnel(self):
        self.move_for_duration(0,0.5,0.25,0,2.5)

    def go_tunnelr(self):
        self.move_for_duration(0,-0.5,-0.25,0,2.5)

    def go_sign8(self):
        self.move_for_duration(-1.2,0,2.05,0,1.4)
        self.move_for_duration(0,0,0,0,0.5)
        self.move_for_duration(0,1.8,0,0,2.15)

    def go_sign8r(self):
        self.move_for_duration(0,-1.8,0,0,2.15)
        self.move_for_duration(0,0,0,0,0.5)
        self.move_for_duration(1.2,0,-2.05,0,1.4)



    def operate(self):
        """Main loop to check state and drive accordingly."""
        while not rospy.is_shutdown():
            if self.state == "STARTUP":
                #self.update_velocity(0,0,0.01,0)
                rospy.loginfo("Starting up...")

            elif self.state == "DRIVING":
                if self.clue_count == 0:
                    if self.leaving_prev_clue: # Pre operation
                        self.go_sign1()
                        self.leaving_prev_clue = False

                    if self.clue_searching: # Post operation
                        #rospy.loginfo("Searching for clue 1")
                        self.update_velocity(0,0,0,0)
                        #rospy.sleep(1)
                        continue

                    else: # Regular operation
                        self.update_velocity(0.0,0,-0.01,0)


                elif self.clue_count == 1:

                    if self.leaving_prev_clue: # Pre operation
                        self.go_sign2()
                        self.leaving_prev_clue = False

                    if self.clue_searching: # Post operation
                        self.update_velocity(-0.05,0,0,0)

                    else: # Regular operation
                        self.update_velocity(0,0,0,0)

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
    driving.listen_for_commands()
    #driving.start()