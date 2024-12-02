#!/usr/bin/env python3

import rospy
import numpy as np
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist

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

def update_velocity(forward, sideways, up, yaw):
    vx = forward * 0.5  # Forward/Backward
    vy = sideways * 0.5  # Left/Right (corrected)
    vz = up * 0.5  # Up/Down
    vaz = yaw  # Rotate left/right

    vel_msg = Twist()
    vel_msg.linear.x = vx
    vel_msg.linear.y = vy
    vel_msg.linear.z = vz
    vel_msg.angular.z = vaz
    # rospy.loginfo("Publishing New Velocity")
    vel_pub.publish(vel_msg)

def respawn():
    spawn_position([5.52, 2.3, 0, 0, 0, np.sin(-np.pi/4), np.cos(-np.pi/4)])
    rospy.sleep(0.1)    
    update_velocity(0,0,0,0)


    rospy.loginfo("Respawn Complete")

if __name__ == '__main__':
    rospy.init_node('quadrotor_controller', anonymous=True)
    vel_pub = rospy.Publisher('/quad/cmd_vel', Twist, queue_size=1)
    respawn()