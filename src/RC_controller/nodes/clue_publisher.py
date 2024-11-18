#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int8

class CluePublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('clue_publisher', anonymous=True)

        # Initialize the publisher for score tracker topic
        self.pub = rospy.Publisher('/score_tracker', String, queue_size=10)

        # Subscribe to the OCR output
        rospy.Subscriber('/verified_clues', String, self.verified_clues_callback)

        # Subscribe to the State output
        rospy.Subscriber('/state', String, self.state_callback)

        # Subscribe to the Clue Count output
        rospy.Subscriber('/clue_count', Int8, self.clue_count_callback)

        self.clue_count = 0


    def start_comp(self):
        self.pub.publish(str('TeamRC,funky,0,START'))

    def end_comp(self):
        self.pub.publish(str('TeamRC,funky,-1,END'))

    def submit_clue(self, clue, answer):
        number = str(clue)
        answer = str(answer)
        self.pub.publish(f'TeamRC,funky,{number},{answer}')

    def clue_count_callback(self, count):
        self.clue_count = count.data

    def verified_clues_callback(self, clue):
        self.submit_clue(clue, self.clue_count)

    def state_callback(self, state):
        if state.data == "STARTUP":
            self.start_comp()

    def start(self):
        # Start the ROS loop
        rospy.spin()

if __name__ == '__main__':
    # Create an instance of the CluePublisher class
    clue_publisher = CluePublisher()
    rospy.loginfo("Clue Verification Node Initialized.")
    clue_publisher.start()