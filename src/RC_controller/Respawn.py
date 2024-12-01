#!/usr/bin/env python3

import rospy
import numpy as np
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

def spawn_position(position):
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

def respawn():
    spawn_position([5.52, 2.3, 0, 0, 0, np.sin(-np.pi/4), np.cos(-np.pi/4)])
    rospy.loginfo("Respawn Complete")

if __name__ == '__main__':
    respawn()