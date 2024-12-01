#!/usr/bin/env python3
import sys
import rospy
from PyQt5 import QtWidgets, QtGui, QtCore, uic
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

ui_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "RC_App.ui")
icon_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icon.png")


class MainWindow(QtWidgets.QMainWindow):
    """
    Main window class that loads the UI and updates QLabel placeholders with camera feeds.
    """
    def __init__(self):
        super().__init__()

        self.setWindowIcon(QtGui.QIcon(icon_path))

        # Load the UI file
        uic.loadUi(ui_path, self)

        # Initialize ROS node
        rospy.init_node('camera_feed_gui', anonymous=True)

        # Initialize the CvBridge for ROS Image message conversion
        self.bridge = CvBridge()

        # Define camera topics and corresponding QLabel placeholders
        self.camera_topics = {
            "front_cam_l": "/quad/front_cam/camera/image",
            "back_cam_l": "/quad/back_cam/back_camera/image",
            "left_cam_l": "/quad/left_cam/left_camera/image",
            "right_cam_l": "/quad/right_cam/right_camera/image",
            "down_cam_l": "/quad/downward_cam/down_camera/image"
        }

        # Create subscribers for each camera topic
        self.camera_subscribers = {}
        for label_name, topic in self.camera_topics.items():
            label = self.findChild(QtWidgets.QLabel, label_name)
            if label:
                self.camera_subscribers[topic] = rospy.Subscriber(
                    topic, Image, self.create_image_callback(label)
                )
            else:
                rospy.logerr(f"Placeholder not found: {label_name}")

    def convert_cv_to_pixmap(self, cv_img):
        """
        Converts an OpenCV image (numpy array) to a QPixmap for display in PyQt.
        """
        # Convert BGR to RGB for QImage
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        height, width, channel = cv_img.shape
        bytes_per_line = channel * width
        q_img = QtGui.QImage(cv_img.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)
        return QtGui.QPixmap.fromImage(q_img)

    def create_image_callback(self, label):
        """
        Creates a callback function for a specific QLabel to update with the ROS camera feed.
        """
        def image_callback(msg):
            try:
                # Convert ROS Image message to OpenCV format
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

                # Convert OpenCV image to QPixmap
                pixmap = self.convert_cv_to_pixmap(cv_img)

                # Scale the pixmap to fit the QLabel's size while keeping the aspect ratio
                scaled_pixmap = pixmap.scaled(
                    label.width(), label.height(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
                )

                # Update the QLabel with the scaled QPixmap
                label.setPixmap(scaled_pixmap)
            except Exception as e:
                rospy.logerr(f"Error processing image for label {label.objectName()}: {e}")

        return image_callback


if __name__ == "__main__":
    # Initialize the PyQt application
    app = QtWidgets.QApplication(sys.argv)

    # Create and show the main window
    window = MainWindow()
    window.show()

    # Execute the application
    sys.exit(app.exec_())
