#!/usr/bin/env python3

import rospy
import cv2
import sys
import os

# Add the path to the 'nodes' directory where sign_reader.py is located
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'nodes'))

from sensor_msgs.msg import Image
from sign_reader import SignReader

def main():
    # Create an instance of the SignReader class and start it
    sign_reader = SignReader()
    sign_reader.start()

if __name__ == '__main__':
    main()