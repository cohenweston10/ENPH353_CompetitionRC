#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int8
import ast

class CluePublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('clue_publisher', anonymous=True)

        # Initialize the publisher for score tracker topic
        self.pub = rospy.Publisher('/score_tracker', String, queue_size=10)

        # Initialize the publisher for clue count topic
        self.pub_count = rospy.Publisher('/clue_count', Int8, queue_size=10)

        # Subscribe to the OCR output
        rospy.Subscriber('/ocr/processed_strings', String, self.callback)

        # Subscribe to the State output
        rospy.Subscriber('/state', String, self.state_callback)

        self.publish_count = 0
        self.pub_count.publish(self.publish_count)

        self.ClueToNum = {
            "SIZE": 1, "VICTIM": 2, "CRIME": 3, "TIME": 4,
            "PLACE": 5, "MOTIVE": 6, "WEAPON": 7, "BANDIT": 8
        }

    def callback(self, clue):
        clues = ast.literal_eval(clue.data)
        if clues[0] in self.ClueToNum:
            self.submit_clue(self.ClueToNum[clues[0]], clues[1])
            del self.ClueToNum[clues[0]]

    def start_comp(self):
        self.pub.publish(str('TeamRC,funky,0,START'))

    def end_comp(self):
        self.pub.publish(str('TeamRC,funky,-1,END'))

    def submit_clue(self, clue, answer):
        self.pub.publish(f'TeamRC,funky,{clue},{answer}')
        self.publish_count += 1
        self.pub_count.publish(self.publish_count)
        rospy.loginfo(f"Sent {clue},{answer} to score tracker")



    def state_callback(self, state):
        if state.data == "STARTUP":
            rospy.loginfo("Starting Comp")
            self.start_comp()
        elif state.data == "STOP":
            rospy.loginfo("Ending Comp")
            self.end_comp()

    def start(self):
        # Start the ROS loop
        rospy.spin()

if __name__ == '__main__':
    # Create an instance of the CluePublisher class
    clue_publisher = CluePublisher()
    rospy.loginfo("Clue Verification Node Initialized.")
    clue_publisher.start()