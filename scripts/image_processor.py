#!/usr/bin/env
import cv_bridge
import numpy as np
import rospy
from cv2 import cv2
from sensor_msgs.msg import Image


class ImageProcessor:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(
            "camera/rgb/image_raw", Image, self.image_callback
        )

        self.image = None

        red = np.uint8([[[0, 0, 255]]])
        green = np.uint8([[[0, 255, 0]]])
        blue = np.uint8([[[255, 0, 0]]])
        yellow = np.uint8([[[0, 255, 255]]])

        colors = [red, green, blue, yellow]
        color_names = ["red", "green", "blue", "yellow"]

        self.color_map = {}
        for c, c_name in zip(colors, color_names):
            hsv_c = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
            lower_color = np.array([hsv_c[0][0][0] - 10, 100, 100])
            upper_color = np.array([hsv_c[0][0][0] + 10, 255, 255])
            self.color_map[c_name] = (lower_color, upper_color)

    def image_callback(self, data: Image):
        self.image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

    def get_center_for_color(self, color: str) -> float:
        """Gets the center of the image for the given color.

        Parameters:
            color: one of "red", "green", "blue", or "yellow"
        Returns:
            The x coordinate of the center of the color in the current image.
            If no pixels of the color are found, returns -1.
        """

        if color not in self.color_map:
            print(f"Color {color} not found")
            return

        while self.image is None:
            pass

        image = self.image
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_color, upper_color = self.color_map[color]

        h, w, d1 = image.shape
        search_top = int(h / 2)
        search_bot = int(h / 2 + 1)

        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Erase all pixels that aren't the correct color
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # Determine the center of the dumbbell
        M = cv2.moments(mask)
        # Get the center of the dumbbell if color pixels are found
        if M["m00"] > 0:
            # center of the colored pixels in the image
            return M["m10"] / M["m00"] - w / 2

        return float("inf")

    def get_y_center_for_color(self, color: str) -> float:
        """Gets the y-center of the image for the given color.

        Parameters:
            color: one of "red", "green", "blue", or "yellow"
        Returns:
            The x coordinate of the center of the color in the current image.
            If no pixels of the color are found, returns -1.
        """

        if color not in self.color_map:
            print(f"Color {color} not found")
            return

        while self.image is None:
            pass

        image = self.image
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_color, upper_color = self.color_map[color]

        h, w, d1 = image.shape
        search_top = int(h / 2)
        search_bot = int(h / 2 + 1)

        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Erase all pixels that aren't the correct color
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # Determine the center of the dumbbell
        M = cv2.moments(mask)
        # Get the center of the dumbbell if color pixels are found
        if M["m00"] > 0:
            # center of the colored pixels in the image
            return M["m01"] / M["m00"] - h / 2

        return float("inf")
