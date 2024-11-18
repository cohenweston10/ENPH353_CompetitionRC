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

        # Initialize the publisher for clue count topic
        self.pub_count = rospy.Publisher('/clue_count', Int8, queue_size=10)

        # Subscribe to the OCR output
        rospy.Subscriber('/verified_clues', String, self.verified_clues_callback)

        # Subscribe to the State output
        rospy.Subscriber('/state', String, self.state_callback)

        self.publish_count = 0
        self.pub_count.publish(self.publish_count)


    def start_comp(self):
        self.pub.publish(str('TeamRC,funky,0,START'))

    def end_comp(self):
        self.pub.publish(str('TeamRC,funky,-1,END'))

    def submit_clue(self, clue, answer):
        number = str(clue + 1)
        answer = str(answer.data)
        self.pub.publish(f'TeamRC,funky,{number},{answer}')
        self.publish_count += 1
        self.pub_count.publish(self.publish_count)
        rospy.loginfo(f"Sent {number},{answer} to score tracker")

    def verified_clues_callback(self, clue):
        self.submit_clue(self.publish_count, clue)

    def state_callback(self, state):
        rospy.loginfo("Trying to start comp.")
        if state.data == "STARTUP":
            self.start_comp()
            rospy.loginfo("Starting Comp.")

    def start(self):
        # Start the ROS loop
        rospy.spin()

if __name__ == '__main__':
    # Create an instance of the CluePublisher class
    clue_publisher = CluePublisher()
    rospy.loginfo("Clue Verification Node Initialized.")
    clue_publisher.start()