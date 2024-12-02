#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int8
import ast

class ClueVerifier:
    def __init__(self):
        # Initialize the node
        rospy.init_node('clue_verifier', anonymous=True)

        # Initialize the publisher for score tracker topic
        self.pub_score = rospy.Publisher('/verified_clues', String, queue_size=10)

        # Subscribe to the Clue Count output
        rospy.Subscriber('/clue_count', Int8, self.clue_count_callback)

        # Subscribe to the OCR output
        rospy.Subscriber('/ocr/processed_strings', String, self.callback)

        self.ClueToNum = {
            "SIZE": 1, "VICTIM": 2, "CRIME": 3, "TIME": 4,
            "PLACE": 5, "MOTIVE": 6, "WEAPON": 7, "BANDIT": 8
        }

        self.clue_count = 0


    def callback(self, clue):
        clues = ast.literal_eval(clue.data)
        if clues[0] in self.ClueToNum.keys():
            self.pub_score.publish(clues)

    def clue_count_callback(self, count):
        self.clue_count = count.data

    def start(self):
        # Start the ROS loop
        rospy.spin()

if __name__ == '__main__':
    # Create an instance of the ClueVerifier class
    clue_verifier = ClueVerifier()
    rospy.loginfo("Clue Publisher Node Initialized.")
    clue_verifier.start()