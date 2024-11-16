#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String

class CluePublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('clue_publisher', anonymous=True)

        # Initialize the publisher for score tracker topic
        self.pub = rospy.Publisher('/score_tracker', String, queue_size=10)

        # Subscribe to the OCR output
        rospy.Subscriber('/ocr/processed_strings', String, queue_size=10)


    def start_comp(self):
        self.pub.publish(str('TeamRC,funky,0,START'))

    def end_comp(self):
        self.pub.publish(str('TeamRC,funky,-1,END'))

    def submit_clue(self, clue, answer):
        number = str(clue)
        answer = str(answer)
        self.pub.publish(f'TeamRC,funky,{number},{answer}')

    def start(self):
        # Start the ROS loop
        rospy.spin()

if __name__ == '__main__':
    # Create an instance of the CluePublisher class
    clue_publisher = CluePublisher()
    clue_publisher.start()