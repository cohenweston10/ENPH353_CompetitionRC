#!/usr/bin/env python3

import rospy
import cv2
import sys
import os

# Add the path to the 'nodes' directory where sign_reader.py is located
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'nodes'))

from sensor_msgs.msg import Image
from sign_reader import SignReader
from character_recognition import OCRNode
from clue_verification import ClueVerifier
from clue_publisher import CluePublisher

def main():
    # Create an instance of the SignReader class and start it
    sign_reader = SignReader()
    sign_reader.start()

    # TO BE TESTED #
    # Create an instance of the OCR class and display latest image
    parser = OCRNode()
    cv2.imshow("Sign", parser.latest_image)
    cv2.waitKey(1)




if __name__ == '__main__':
    main()