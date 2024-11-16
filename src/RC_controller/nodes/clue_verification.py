#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
import ast

class ClueVerifier:
    def __init__(self):
        # Initialize the node
        rospy.init_node('clue_verifier', anonymous=True)

        # Initialize the publisher for score tracker topic
        self.pub = rospy.Publisher('/verified_clues', String, queue_size=10)

        # Subscribe to the OCR output
        rospy.Subscriber('/ocr/processed_strings', String, self.callback)

        self.ClueToNum = {
            1: "SIZE", 2: "VICTIM", 3: "CRIME", 4: "TIME",
            5: "PLACE", 6: "MOTIVE", 7: "WEAPON", 8: "BANDIT"
        }

        self.publish_count = 0

    def callback(self, clue):
        clues = ast.literal_eval(clue.data)
        if clues[0] == self.ClueToNum.get(self.publish_count + 1):
            self.pub.publish(clues[1])
            self.publish_count += 1

    def start(self):
        # Start the ROS loop
        rospy.spin()

if __name__ == '__main__':
    # Create an instance of the ClueVerifier class
    clue_verifier = ClueVerifier()
    rospy.loginfo("Clue Publisher Node Initialized.")
    clue_verifier.start()