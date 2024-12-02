#!/usr/bin/env python3

import rospy
import cv2
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import numpy as np
import roslaunch
import subprocess


class CompProtocol:
    def __init__(self):
        rospy.init_node('competition_protocol_node')
        self.pub = rospy.Publisher('/state', String, queue_size=10)
        rospy.Subscriber('/clue_count', Int8, self.clue_count_callback)

        self.clue_count = 0

    def clue_count_callback(self, count):
        self.clue_count = count.data

    def spawn_position(self, position):
        msg = ModelState()
        msg.model_name = 'quadrotor'

        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.x = position[3]
        msg.pose.orientation.y = position[4]
        msg.pose.orientation.z = position[5]
        msg.pose.orientation.w = position[6]

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( msg )

        except rospy.ServiceException:
            print ("Service call failed")

    def respawn(self):
        self.spawn_position([5.52, 2.3, 0, 0, 0, np.sin(-np.pi/4), np.cos(-np.pi/4)])
        rospy.loginfo("Respawn Complete")


    def run(self):
        rospy.sleep(1)
        self.pub.publish("STARTUP")
        rospy.loginfo("Published STARTUP state to /state")

        rospy.sleep(1)
        self.pub.publish("DRIVING")
        rospy.loginfo("Published DRIVING state to /state")

        drive_time = 0
        while self.clue_count < 8:
            rospy.sleep(1)
            drive_time += 1
            # if drive_time >= 30:
            #     break

        self.pub.publish("STOP")
        rospy.loginfo("Published STOP state to /state")

        self.respawn()


if __name__ == '__main__':
    comp_protocol = CompProtocol()
    comp_protocol.run()