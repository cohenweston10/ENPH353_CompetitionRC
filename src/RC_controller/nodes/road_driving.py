#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image




THRESHOLD = 200 #value is based off of lab 2

TOPIC_CONTROL = '/quad/cmd_vel'
TOPIC_IMAGE_FEED = '/quad/downward_cam/down_camera/image'

LINEAR_SPEED = 0.5
ANGULAR_SPEED = 0.5

SCAN_HEIGHT = 400

#return the center coordinate of the road
#if no road is detected, return -1
def getRoadCenterCoord(frame):
  grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  frameHeight = int(grayFrame.shape[0])

  #Record the horizontal positions of any pixels below the threshold
  roadValues = np.array(np.where(grayFrame[frameHeight - SCAN_HEIGHT] > THRESHOLD))

  #Update the circle center coordinate for the new frame unless no road is detected
  if not roadValues.size == 0:
    roadCenter = np.mean(roadValues)
  #print(roadCenter)
  else:
    roadCenter = -1

  return roadCenter


class RoadDriving:
    def __init__(self):
        # Initialize the node
        rospy.init_node('road_driving', anonymous=True)

        self.vel_pub = rospy.Publisher(TOPIC_CONTROL, Twist, queue_size=1)

        # Subscribe to the State output
        rospy.Subscriber('/state', String, self.state_callback)

        # Subscribe to the Clue Count output
        rospy.Subscriber('/clue_count', Int8, self.clue_count_callback)


        self.bridge = CvBridge()
        self.rate = rospy.Rate(60)

        self.image_sub = rospy.Subscriber(TOPIC_IMAGE_FEED,Image,self.image_callback)

        self.state = ""
        self.clue_count = 0
        self.switch_pending = True
        self.image = None

    def image_callback(self, image):
        try:
            # Convert the ROS image message to OpenCV format with the correct encoding
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            # Ensure the image is in the correct depth
            self.image = cv_image.astype(np.uint8)        
        except CvBridgeError as e:
            rospy.loginfo(e)
            return
        

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
                self.update_velocity(0,0,0.35,0)

            elif self.state == "DRIVING":
                if self.clue_count == 0:
                    while self.image is None: #wait for an image to be received
                        ()
                    self.lineFollow()
                elif self.clue_count == 1:
                    self.lineFollow()


            elif self.state == "STOP":
                self.update_velocity(0, 0, 0, 0)
            rospy.sleep(0.1)  # Sleep for a short time to avoid high CPU usage


    def sideSwap(self, direction):
        if direction == "ToLeft":
            self.update_velocity(0,0.3,0,0)
            rospy.sleep(1)
            self.update_velocity(0, 0, 0, 0)
        elif direction == "ToRight":
            self.update_velocity(0,-0.3,0,0)
            rospy.sleep(1)
            self.update_velocity(0, 0, 0, 0)

    def lineFollow(self):

        cv_image = self.image

        # cv2.imshow("Raw Image", cv_image)
        # cv2.waitKey(1)


        frameHeight = cv_image.shape[0]
        frameWidth = cv_image.shape[1]
        seeRoad = False
        roadToLeft = 1
        frameCenter = float(frameWidth) / 2

        roadCenterCoord = getRoadCenterCoord(cv_image)
        #rospy.loginfo(roadCenterCoord)
        if roadCenterCoord > frameCenter:
            roadToLeft = -1

        # if seeRoad remains false, the robot will just rotate in place until it finds the road
        # roation direction is determined by roadToLeft truth value
        if not roadCenterCoord == -1:
            seeRoad = -1

        # draw a circle on the image, just like lab 2 (for debugging/visual feedback)
        try:
            imgWithCircle = cv2.circle(cv_image,(int(roadCenterCoord),int(frameHeight) - SCAN_HEIGHT), 15, (255,0,255), -1)

            cv2.imshow("Processed Image", imgWithCircle)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.loginfo(e)
            return

        # if the robot does not see road, it will roate in place
        if roadCenterCoord == -1:
            angZ = roadToLeft * ANGULAR_SPEED * -1
            self.update_velocity(0,0,0,angZ)
        else:
            # scale the rotation speed depending on how far the road is from the center
            distanceFromCenter = np.abs(frameCenter - roadCenterCoord)
            proportionAwayFromCenter = np.abs(float(distanceFromCenter) / float(frameCenter))
            angZ = roadToLeft * proportionAwayFromCenter * ANGULAR_SPEED * -1

            # if the road moves away fromt the camera center, robot will slow down
            linX = LINEAR_SPEED * (1-proportionAwayFromCenter**0.5)

            self.update_velocity(linX,0,0,angZ)

    def start(self):
        # Start the ROS loop
        self.operate()

if __name__ == '__main__':
    # Create an instance of the RoadDriving class
    road_driving = RoadDriving()
    rospy.loginfo("Road Driving Node Initialized")
    road_driving.start()