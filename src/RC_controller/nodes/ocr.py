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
from collections import Counter


class OCRNode:
    def __init__(self):
        """
        Initialize the OCRNode.
        """
        model_path = os.path.dirname(os.path.realpath(__file__)) + "/letsgo.keras"

        self.PLATE_HEIGHT = 400
        self.PLATE_WIDTH = 600

        self.char_width = 45
        self.char_height = 70

        self.padding = 3

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
            # cv2.imshow("Received Image", self.latest_image)
            # cv2.waitKey(1)

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
        Dynamically extract character regions from the board image, while filtering out non-character regions like graphics.
        """
        # Convert the board to grayscale if it's not already
        if board.ndim == 3 and board.shape[2] == 3:  
            board = cv2.cvtColor(board, cv2.COLOR_BGR2GRAY)

        # Normalize the board and convert to uint8 for processing
        board = board.astype(np.float32) / 255.0
        board_uint8 = (board * 255).astype(np.uint8)

        # Determine the dominant background color
        flat_pixels = board_uint8.flatten()
        background_color = int(Counter(flat_pixels).most_common(1)[0][0])  # Most frequent pixel value

        # Threshold the image to isolate characters
        _, binary = cv2.threshold(board_uint8, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # Mask the known graphic region (update coordinates as needed)
        mask = np.ones(binary.shape, dtype=np.uint8) * 255
        graphic_region1 = (0, 0, 240, 200)  # Replace with actual region of the graphic
        graphic_region2 = (240, 150, 280, 200)  # Replace with actual region of the graphic
        cv2.rectangle(mask, (graphic_region1[0], graphic_region1[1]), (graphic_region1[2], graphic_region1[3]), 0, -1)
        cv2.rectangle(mask, (graphic_region2[0], graphic_region2[1]), (graphic_region2[2], graphic_region2[3]), 0, -1)
        binary = cv2.bitwise_and(binary, binary, mask=mask)

        # cv2.imshow("mask", binary)
        # cv2.waitKey(1)

        # Perform morphological operations to clean up noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary_cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # Find contours of the characters
        contours, _ = cv2.findContours(binary_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours by position (top-to-bottom, left-to-right)
        bounding_boxes = [cv2.boundingRect(c) for c in contours]
        bounding_boxes = sorted(bounding_boxes, key=lambda b: (b[1], b[0]))  # Sort by y first, then x

        # Separate top and bottom rows based on their y-coordinates
        row_threshold = board.shape[0] // 2  # Split roughly in half vertically
        top_row_boxes = []
        bottom_row_boxes = []

        for box in bounding_boxes:
            x, y, w, h = box
            if h < 20 or w < 10:  # Skip very small boxes (likely noise)
                continue
            if w > self.char_width * 2 or h > self.char_height * 2:  # Skip large contours
                continue
            aspect_ratio = w / h
            if aspect_ratio < 0.2 or aspect_ratio > 1.5:  # Skip non-character shapes
                continue
            if y < row_threshold:
                top_row_boxes.append(box)
            else:
                bottom_row_boxes.append(box)

        # Sort each row by x-coordinate
        top_row_boxes = sorted(top_row_boxes, key=lambda b: b[0])
        bottom_row_boxes = sorted(bottom_row_boxes, key=lambda b: b[0])

        # Extract and preprocess characters
        def extract_characters(row_boxes):
            chars = []
            for x, y, w, h in row_boxes:
                char_im = board_uint8[y:y + h, x:x + w]
                # Add padding around the character
                char_im = cv2.copyMakeBorder(
                    char_im,
                    top=self.padding, bottom=self.padding, left=self.padding, right=self.padding,
                    borderType=cv2.BORDER_CONSTANT,
                    value=background_color  # Use dynamically determined background color
                )
                # Resize to target dimensions
                char_im = cv2.resize(char_im, (self.char_width, self.char_height), interpolation=cv2.INTER_AREA)
                chars.append(char_im)
            return chars

        # Process top and bottom rows
        top_chars = extract_characters(top_row_boxes)
        bottom_chars = extract_characters(bottom_row_boxes)

        return top_chars, bottom_chars


    @staticmethod
    def find_actual_char(predictions):
        """
        Finds the actual character based on the model's predictions.
        """
        characters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
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
        # for idx, char_img in enumerate(test_top):
        #     cv2.imshow(f"Top Character {idx+1}", char_img.squeeze())
        # for idx, char_img in enumerate(test_bottom):
        #     cv2.imshow(f"Bottom Character {idx+1}", char_img.squeeze())
        # cv2.waitKey(1)

        # Convert to NumPy arrays for prediction
        test_top_np = np.array(test_top, dtype=np.float32) / 255.0
        test_bottom_np = np.array(test_bottom, dtype=np.float32) / 255.0

        # Debug: Log shapes and first elements
        # rospy.loginfo(f"Top Characters Shape: {test_top_np.shape}")
        # rospy.loginfo(f"Bottom Characters Shape: {test_bottom_np.shape}")

        # Predict characters
        top_string = self.model.predict(test_top_np, verbose=0)
        bottom_string = self.model.predict(test_bottom_np, verbose=0)

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
        # rospy.loginfo("Publishing result: %s", result)

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
