#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time


THRESHOLD = 50 #value is based off of lab 2

TOPIC_CONTROL = '/quad/cmd_vel'
TOPIC_IMAGE_FEED = '/quad/downward_cam/down_camera/image'

LINEAR_SPEED = 4
ANGULAR_SPEED = 1

MAX_ANGULAR_SPEED = 5

SCAN_HEIGHT = 479

#return the center coordinate of the road
#if no road is detected, return -1
def getRoadCenterCoord(frame, scan_height):
  grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  frameHeight = int(frame.shape[0])

  #Record the horizontal positions of any pixels below the threshold
  roadValues = np.where(grayFrame[frameHeight - scan_height - 1] > THRESHOLD)[0]
  # rospy.loginfo(f"There are {len(roadValues)} road pixels out of {frame.shape[1]} pixels in the scan row.")

  rightRoadValues = roadValues[roadValues > frame.shape[1]/2]

  if rightRoadValues.shape[0] >= frame.shape[1]/2 -1:
    rightTurn = True
  else:
    rightTurn = False





  #Update the circle center coordinate for the new frame unless no road is detected
  if not roadValues.size == 0:
    roadCenter = np.mean(roadValues)
  #print(roadCenter)
  else:
    roadCenter = -1

  return roadCenter, rightTurn


class RoadDriving:
    def __init__(self):
        # Initialize the node
        rospy.init_node('road_driving', anonymous=True)

        self.vel_pub = rospy.Publisher(TOPIC_CONTROL, Twist, queue_size=1)

        # Subscribe t        self.switch_pending = Trueo the State output
        rospy.Subscriber('/state', String, self.state_callback)

        # Subscribe to the Clue Count output
        rospy.Subscriber('/clue_count', Int8, self.clue_count_callback)

        # Subscribe to the OCR output
        rospy.Subscriber('/ocr/processed_strings', String, self.OCRcallback)


        self.bridge = CvBridge()
        self.rate = rospy.Rate(60)

        self.image_sub = rospy.Subscriber(TOPIC_IMAGE_FEED,Image,self.image_callback)

        self.state = ""
        self.clue_count = 0
        self.image = None
        self.switch_pending = True
        self.clue_searching = False
        self.last_clue = -5
        self.prev_error = 0
        self.integral = 0

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
        self.clue_searching = False
        self.last_clue = time.time()

    def OCRcallback(self, data):
        if time.time() - self.last_clue > 75:
            self.clue_searching = True
            rospy.loginfo(f"Searching for clue {self.clue_count + 1}, time since last clue is {time.time() - self.last_clue}")

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
                self.update_velocity(0,0,0.5,0)

            elif self.state == "DRIVING":
                self.lineFollow()

            elif self.state == "DRIVING1": ##TESTING##

                if self.clue_count == 0:
                    if self.switch_pending == True: # Pre operation
                        rospy.loginfo("Swapping Left")
                        self.sideSwap("ToLeft")
                        rospy.sleep(1)

                        self.switch_pending = False

                    if self.clue_searching == True: # Post operation
                        self.update_velocity(0.1,0,0,0)

                    else: # Regular operation
                        self.update_velocity(0.2,0,0,0)


                elif self.clue_count == 1:
                    if self.switch_pending == True: # Pre operation
                        rospy.loginfo("Swapping Right")
                        self.sideSwap("ToRight")
                        rospy.sleep(3)

                        self.switch_pending = False
                        
                    if self.clue_searching == True: # Post operation
                        self.update_velocity(0.1,0,0,0)

                    else: # Regular operation
                        self.update_velocity(0.65,0,0,0) 



                elif self.clue_count == 2:
                    if self.switch_pending == True: # Pre operation
                        rospy.loginfo("Doing state 2 pre-operation")
                        self.localize2()

                        self.switch_pending = False

                    if self.clue_searching == True: # Post operation
                        # TODO state 2 post-operation
                        rospy.loginfo("Looking for sign 3")

                    else:
                        # TODO state 2 regular operation
                        rospy.loginfo("In regular state 2 operation")

                elif self.clue_count == 3:
                    self.lineFollow()

                elif self.clue_count == 4:
                    self.lineFollow()

                elif self.clue_count == 5:
                    self.lineFollow()

                elif self.clue_count == 6:
                    self.lineFollow()

                elif self.clue_count == 7:
                    self.lineFollow()

                elif self.clue_count == 8:
                    self.lineFollow()

            elif self.state == "STOP":
                self.update_velocity(0, 0, 0, 0)
                self.switch_pending = True

            rospy.sleep(0.1)  # Sleep for a short time to avoid high CPU usage


    def sideSwap(self, direction):
        if direction == "ToLeft":
            self.update_velocity(0,0.3,0,0)
        elif direction == "ToRight":
            self.update_velocity(0,-0.3,0,0)

    def lineFollow(self):

        cv_image = self.image

        # cv2.imshow("Raw Image", cv_image)
        # cv2.waitKey(1)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the color range for the shape
        lower_color = np.array([0, 0, 50])  # Adjust for the target color
        upper_color = np.array([180, 50, 200])

        # Create a mask for the color
        mask = cv2.inRange(hsv, lower_color, upper_color)
        filtered = cv2.bitwise_and(cv_image, cv_image, mask=mask)


        frameHeight = filtered.shape[0]
        frameWidth = filtered.shape[1]
        seeRoad = False
        roadToLeft = 1
        frameCenter = float(frameWidth) / 2

        upperRoadCenterCoord, rightTurn = getRoadCenterCoord(filtered, SCAN_HEIGHT)
        lowerRoadCenterCoord, _ = getRoadCenterCoord(filtered, 0)
        #rospy.loginfo(roadCenterCoord)
        if upperRoadCenterCoord > lowerRoadCenterCoord:
            roadToLeft = -1
            # rospy.loginfo("Road to the right, rotating left.")
        else:
            roadToLeft = 1
            # rospy.loginfo("Road to the left, rotating right.")

        # if seeRoad remains false, the robot will just rotate in place until it finds the road
        # roation direction is determined by roadToLeft truth value
        if upperRoadCenterCoord != -1:
            seeRoad = True

        # draw a circle on the image, just like lab 2 (for debugging/visual feedback)
        try:
            imgWithCircle1 = cv2.circle(filtered,(int(upperRoadCenterCoord),int(frameHeight) - SCAN_HEIGHT - 1), 15, (255,0,255), -1)
            imgWithCircle = cv2.circle(filtered,(int(lowerRoadCenterCoord),int(frameHeight) - 1), 15, (255,0,255), -1)

            cv2.imshow("Processed Image", imgWithCircle)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.loginfo(e)
            return

        # if the robot does not see road, it will roate in place
        if upperRoadCenterCoord == -1:
            angZ = roadToLeft * ANGULAR_SPEED * -1
            self.update_velocity(0,0,0,angZ)
        else:
            # scale the rotation speed depending on how far the road is from the center
            distanceFromCenter = np.abs(frameCenter - upperRoadCenterCoord)
            proportionAwayFromCenter = np.abs(float(distanceFromCenter) / float(frameCenter))
            # frontBackdistance = np.abs(float(upperRoadCenterCoord) - float(lowerRoadCenterCoord))
            # angZ = roadToLeft * np.exp(frontBackdistance/10) * proportionAwayFromCenter * ANGULAR_SPEED * 1/10

            error = frameCenter - upperRoadCenterCoord
            angZ = 0.01 * error + 0.0001 * self.integral + 0.1 * (error - self.prev_error)
            self.prev_error = error
            self.integral += error

            angZ = np.clip(angZ, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

            # rospy.loginfo(f"Angular speed: {angZ}, Error: {error}, Previous Error: {self.prev_error}, Integral: {self.integral}")


            # if the road moves away fromt the camera center, robot will slow down
            linX = LINEAR_SPEED * (1-proportionAwayFromCenter**0.2)

            linX = np.clip(linX, 0, 1)

            # if rightTurn:
            #     angZ = -MAX_ANGULAR_SPEED
            #     linX = 0.1

            self.update_velocity(linX,0,0,angZ)

    def localize2(self):
        centered = False

        while (not centered):
            cv_image = self.image 

            if cv_image is None or cv_image.size == 0:
                return     

            # It converts the BGR color space of image to HSV color space 
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) 

            # Threshold of blue in HSV space 
            lower_blue = np.array([80, 50, 160]) 
            upper_blue = np.array([180, 255, 255])

            # preparing the mask to overlay 
            mask = cv2.inRange(hsv, lower_blue, upper_blue) 

            # The black region in the mask has the value of 0, 
            # so when multiplied with original image removes all non-blue regions 
            blue = cv2.bitwise_and(cv_image, cv_image, mask = mask) 

            blueHeight = blue.shape[0]
            blueWidth = blue.shape[1]

            midHeight = blueHeight // 2
            midWidth = blueWidth // 2

            horizontal = np.array(np.where(blue[midHeight] > 10))
            hloc = sum(horizontal) / len(horizontal)
            vertical = np.array(np.where(blue[:][midWidth] > 10))
            vloc = sum(vertical) / len(vertical)
            
            range = 25

            if vloc < midHeight - range:
                self.update_velocity(0.5,0,0,0)
                rospy.sleep(0.25)
                self.update_velocity(0,0,0,0)
            elif vloc > midHeight + range: 
                self.update_velocity(-0.5,0,0,0)
                rospy.sleep(0.25)
                self.update_velocity(0,0,0,0)
            elif hloc < midWidth - range:
                self.update_velocity(0,-0.5,0,0)
                rospy.sleep(0.25)
                self.update_velocity(0,0,0,0)
            elif hloc > midWidth + range:
                self.update_velocity(0,0.5,0,0)
                rospy.sleep(0.25)
                self.update_velocity(0,0,0,0)
            else:
                centered = True
                rospy.loginfo("Localized on sign 2")

            rospy.loginfo("Trying to localize on sign 2")

            cv2.imshow("Blue", blue)
            cv2.waitKey(1)  




    def start(self):
        # Start the ROS loop
        self.operate()

if __name__ == '__main__':
    # Create an instance of the RoadDriving class
    road_driving = RoadDriving()
    rospy.loginfo("Road Driving Node Initialized")
    road_driving.start()