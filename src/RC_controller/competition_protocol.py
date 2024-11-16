#!/usr/bin/env python3

import rospy
import cv2
import sys
from sensor_msgs.msg import Image

class CompProtocol:
    def __init__(self):
        rospy.init_node('competition_protocol_node')
        self.pub = rospy.Publisher('/state', String, queue_size=10)
        rospy.Subscriber('/clue_count', Int, self.clue_count_callback)

        self.clue_count = 0

    def clue_count_callback(self, count):
        self.clue_count = count

    def run(self):
        self.pub.publish("STARTUP")


if __name__ == '__main__':
    comp_protocol = CompProtocol()
    comp_protocol.run()