#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8

class RoadDriving:
    def __init__(self):
        # Initialize the node
        rospy.init_node('road_driving', anonymous=True)

        self.vel_pub = rospy.Publisher('/quad/cmd_vel', Twist, queue_size=1)

        # Subscribe to the State output
        rospy.Subscriber('/state', String, self.state_callback)

        # Subscribe to the Clue Count output
        rospy.Subscriber('/clue_count', Int8, self.clue_count_callback)

        self.state = ""
        self.clue_count = 0
        self.switch_pending = True

    def state_callback(self, state):
        self.state = state.data

    def clue_count_callback(self, count):
        self.clue_count = count.data
        self.switch_pending = True

    @staticmethod
    def quaternion_to_yaw(orientation):
        """
        Converts quaternion orientation to yaw.
        :param orientation: A quaternion object from Imu topic
        """
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yawZActual = math.atan2(t3, t4)
        if yawZActual < 0:
            yawZActual += 2 * math.pi
        return yawZActual

    def update_velocity(self, forward, sideways, up, yaw):
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
        self.vel_pub.publish(vel_msg)

    def operate(self):
        """Main loop to check state and drive accordingly."""
        while not rospy.is_shutdown():
            if self.state == "STARTUP":
                self.update_velocity(0,0,1,0)

            elif self.state == "DRIVING":
                if self.clue_count == 0:
                    if self.switch_pending == True:
                        rospy.loginfo("Swapping Left")
                        self.sideSwap("ToLeft")
                        self.switch_pending = False
                    self.lineFollow()
                elif self.clue_count == 1:
                    if self.switch_pending == True:
                        rospy.loginfo("Swapping Right")
                        self.sideSwap("ToRight")
                        self.switch_pending = False
                    self.lineFollow()


            elif self.state == "STOP":
                self.update_velocity(0, 0, 0, 0)
            rospy.sleep(0.1)  # Sleep for a short time to avoid high CPU usage


    def sideSwap(self, direction):
        if direction == "ToLeft":
            self.update_velocity(0,0.5,0,0)
            rospy.sleep(1)
            self.update_velocity(0, 0, 0, 0)
        elif direction == "ToRight":
            self.update_velocity(0,-0.5,0,0)
            rospy.sleep(1)
            self.update_velocity(0, 0, 0, 0)

    def lineFollow(self):
        self.update_velocity(0,0,0,0)

    def start(self):
        # Start the ROS loop
        self.operate()

if __name__ == '__main__':
    # Create an instance of the RoadDriving class
    road_driving = RoadDriving()
    rospy.loginfo("Road Driving Node Initialized")
    road_driving.start()