#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from tensorflow.keras import models, layers
from tensorflow import keras


class OCRNode:
    def __init__(self):
        """
        Initialize the OCRNode.
        Args:
            model_path: Path to the trained model.
        """

        model_path = " "

        self.PLATE_HEIGHT = 400
        self.PLATE_WIDTH = 600

        self.topy = 42
        self.topx = 250
        self.bottomy = 263
        self.bottomx = 30

        self.char_width = 45
        self.char_height = 70

        self.bridge = CvBridge()
        self.latest_image = None

        # Load the OCR model
        self.model = self._load_model(model_path)

        # Initialize ROS node, subscribers, and publisher
        rospy.init_node('ocr_node', anonymous=True)
        rospy.Subscriber('/sign_reader/homography', Image, self._sign_reader_callback)
        rospy.Subscriber('/ocr/trigger', String, self._trigger_callback)
        self.result_publisher = rospy.Publisher('/ocr/processed_strings', String, queue_size=10)

        # State to control processing
        self.trigger_received = False

    def _sign_reader_callback(self, msg):
        """
        Callback to update the latest image from the camera topic.
        """
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        self.latest_image = np.expand_dims(self.latest_image, axis=-1)  # Shape becomes (height, width, 1)


    def _trigger_callback(self, msg):
        """
        Callback to handle OCR trigger commands.
        """
        if msg.data.lower() == "process":
            self.trigger_received = True
            rospy.loginfo("OCR trigger received. Processing the latest image.")
            self.process_and_publish()

    def _load_model(self, model_path):
        """
        Load the trained OCR model.
        """
        return keras.models.load_model(model_path)

    @staticmethod
    def convert_to_gray(img):
        """
        Convert an image to grascale.
        """
        grayscale_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        return grayscale_img

    def process_board(self, board):
        """
        Extract character regions from the board image.
        """
        top_chars, bottom_chars = [], []

        for j in range(7):
            char_im = board[self.topy:self.topy + self.char_height,
                            self.topx + self.char_width * j:self.topx + self.char_width * (j + 1)]
            top_chars.append(char_im)

        for i in range(12):
            char_im = board[self.bottomy:self.bottomy + self.char_height,
                            self.bottomx + self.char_width * i:self.bottomx + self.char_width * (i + 1)]
            bottom_chars.append(char_im)

        return np.array(top_chars, dtype=np.float32), np.array(bottom_chars, dtype=np.float32)

    @staticmethod
    def find_actual_char(predictions):
        """
        Finds the actual character based on the model's predictions.
        """
        characters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 "
        index = np.argmax(predictions)
        return characters[index]

    def read_clue(self, board):
        """
        Process the board and predict the characters on it.
        """
        board = np.array(board)
        top, bottom = self.process_board(board)

        top_string = self.model.predict(top)
        bottom_string = self.model.predict(bottom)

        top_string_chars = [self.find_actual_char(pred) for pred in top_string]
        bottom_string_chars = [self.find_actual_char(pred) for pred in bottom_string]

        rospy.loginfo("Top String: %s", ''.join(top_string_chars))
        rospy.loginfo("Bottom String: %s", ''.join(bottom_string_chars))

        return ''.join(top_string_chars), ''.join(bottom_string_chars)


    def process_and_publish(self):
        """
        Process the latest image, predict the characters, and publish the results.
        """
        if self.latest_image is None:
            rospy.logwarn("No image available for processing.")
            return

        # Generate the top and bottom strings
        top_string, bottom_string = self.read_clue(self.latest_image)

        # Create a list of strings to publish
        result = [top_string, bottom_string]

        # Publish the array
        rospy.loginfo("Publishing result: %s", result)
        self.result_publisher.publish(String(data=str(result)))  # Convert list to a string


    def start(self):
        """
        Start the ROS event loop.
        """
        rospy.spin()


if __name__ == "__main__":
    ocr_node = OCRNode('./testmodel3.keras')
    rospy.loginfo("OCR Node Initialized.")
    ocr_node.start()
