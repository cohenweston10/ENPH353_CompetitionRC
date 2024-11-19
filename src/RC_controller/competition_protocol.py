#!/usr/bin/env python3

import rospy
import cv2
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from std_msgs.msg import String

class CompProtocol:
    def __init__(self):
        rospy.init_node('competition_protocol_node')
        self.pub = rospy.Publisher('/state', String, queue_size=10)
        rospy.Subscriber('/clue_count', Int8, self.clue_count_callback)

        self.clue_count = 0

    def clue_count_callback(self, count):
        self.clue_count = count.data

    def run(self):
        rospy.sleep(1)
        self.pub.publish("STARTUP")
        rospy.loginfo("Published STARTUP state to /state")

        rospy.sleep(1)
        self.pub.publish("DRIVING")
        rospy.loginfo("Published DRIVING state to /state")

        rospy.sleep(3)
        self.pub.publish("STOP")
        rospy.loginfo("Published STOP state to /state")

        rospy.spin()


if __name__ == '__main__':
    comp_protocol = CompProtocol()
    comp_protocol.run()