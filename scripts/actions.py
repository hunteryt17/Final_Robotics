#! /usr/bin/env python3
from typing import Optional
import enum

import cv_bridge
import rospy
from cv2 import cv2
from sensor_msgs.msg import Image
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


@enum.unique
class Result(enum.Enum):
    FAILURE = enum.auto()
    SUCCESS = enum.auto()


class ImageProcessor:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(
            "camera/rgb/image_raw", Image, self.image_callback
        )

        self.image = None

        red = np.uint8([[[0, 255, 255]]])
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


class RobotControl:
    def __init__(self):
        rospy.init_node("robodog")
        self.odom = None
        self.ranges = None
        self.image_processor = ImageProcessor()

        self.speed_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_subs = rospy.Subscriber("/odom", Odometry, self.process_odom)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.process_scan)

    def process_odom(self, data: Odometry):
        self.odom = data

    def process_scan(self, data: LaserScan):
        self.ranges = data.ranges

    def set_speed(self, linear_x: float = 0.0, angular_z: float = 0.0):
        speed = Twist()
        speed.linear.x = linear_x
        speed.angular.z = angular_z

        self.speed_pub.publish(speed)

    def get_yaw(self) -> float:
        """Takes in a Pose object and returns yaw."""
        p = self.odom.pose
        yaw = euler_from_quaternion(
            [
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w,
            ]
        )[2]

        return yaw

    def turn_to(self, color: str) -> Result:
        center = float("inf")
        turn_distance = 0
        rate = rospy.Rate(10)
        speed = 0
        max_speed = 0.5

        while abs(center) > 10 and abs(turn_distance) < 2 * np.pi:
            center = self.image_processor.get_center_for_color(color)

            speed = min(abs(speed) + 0.01, max_speed, abs(center * 0.01))
            speed = -speed if center > 0 else speed

            turn_distance += speed / 10
            self.set_speed(angular_z=speed)
            rate.sleep()
            print(f"center: {center}")
        print("DONE")
        self.set_speed()

        if center == float("inf"):
            return Result.FAILURE
        return Result.SUCCESS

    def go_to(self, color: str) -> Result:
        if self.turn_to(color) is Result.FAILURE:
            print(f"Could not find {color}")
            return

        print("1")

        distance = self.ranges[0]
        center = self.image_processor.get_center_for_color(color)

        rate = rospy.Rate(10)
        speed = 0
        max_speed = 1
        while distance > 0.3 or abs(center) > 10:
            center = self.image_processor.get_center_for_color(color)
            distance = self.ranges[0]

            linear = min(speed + 0.05, max_speed, distance * 0.1)
            angular = -center * 0.01
            self.set_speed(linear_x=linear, angular_z=angular)
            rate.sleep()

    def run(self):
        self.go_to("red")
        rospy.sleep(2)
        self.go_to("blue")
        rospy.sleep(2)
        self.go_to("green")
        rospy.sleep(2)
        self.go_to("yellow")


if __name__ == "__main__":
    RobotControl().run()
