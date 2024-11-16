#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from tensorflow.keras import models, layers
from tensorflow import keras
import os


class OCRNode:
    def __init__(self):
        """
        Initialize the OCRNode.
        """
        model_path = os.path.dirname(os.path.realpath(__file__)) + "/testmodel3.keras"

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
        rospy.Subscriber('/signreader/homography', Image, self._sign_reader_callback)
        self.result_publisher = rospy.Publisher('/ocr/processed_strings', String, queue_size=10)

    def _sign_reader_callback(self, msg):
        """
        Callback for receiving images and processing them.
        """
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            self.latest_image = np.expand_dims(self.latest_image, axis=-1)

            # Display the received image
            cv2.imshow("Received Image", self.latest_image)
            cv2.waitKey(1)

            self.process_and_publish()
        except Exception as e:
            rospy.logerr(f"Error in callback: {e}")

    def _load_model(self, model_path):
        """
        Load the trained OCR model.
        """
        return keras.models.load_model(model_path)

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

        return top_chars, bottom_chars

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
        np_current_board = np.array(board)

        # Process the board to extract top and bottom character regions
        test_top, test_bottom = self.process_board(np_current_board)

        # Debug: Display the extracted regions
        for idx, char_img in enumerate(test_top):
            cv2.imshow(f"Top Character {idx+1}", char_img.squeeze())
        for idx, char_img in enumerate(test_bottom):
            cv2.imshow(f"Bottom Character {idx+1}", char_img.squeeze())
        cv2.waitKey(1)

        # Convert to NumPy arrays for prediction
        test_top_np = np.array(test_top, dtype=np.float32) / 255.0
        test_bottom_np = np.array(test_bottom, dtype=np.float32) / 255.0

        # Debug: Log shapes and first elements
        # rospy.loginfo(f"Top Characters Shape: {test_top_np.shape}")
        # rospy.loginfo(f"Bottom Characters Shape: {test_bottom_np.shape}")

        # Predict characters
        top_string = self.model.predict(test_top_np)
        bottom_string = self.model.predict(test_bottom_np)

        # Debug: Log raw predictions
        # rospy.loginfo(f"Top Predictions: {top_string}")
        # rospy.loginfo(f"Bottom Predictions: {bottom_string}")

        # Map predictions to actual characters
        top_string_chars = [self.find_actual_char(element) for element in top_string]
        bottom_string_chars = [self.find_actual_char(element) for element in bottom_string]

        # Debug: Log the results
        rospy.loginfo("Mapped Top String: %s", ''.join(top_string_chars))
        rospy.loginfo("Mapped Bottom String: %s", ''.join(bottom_string_chars))

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

        # Debug: Log the published result
        rospy.loginfo("Publishing result: %s", result)

        # Publish the array
        self.result_publisher.publish(String(data=str(result)))

    def start(self):
        """
        Start the ROS event loop.
        """
        rospy.spin()


if __name__ == "__main__":
    ocr_node = OCRNode()
    rospy.loginfo("OCR Node Initialized.")
    ocr_node.start()
