#! /usr/bin/env python3
import enum
import time

import cv_bridge
import numpy as np
import rospy
from cv2 import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
import moveit_commander
import math

# from robodog.msg import ActionStatus


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


class RobotControl:
    def __init__(self):
        rospy.init_node("robodog")
        self.odom = None
        self.ranges = None
        self.image_processor = ImageProcessor()
        self.arm_manipulator = ArmManipulator()

        self.speed_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_subs = rospy.Subscriber("/odom", Odometry, self.process_odom)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.process_scan)
        # self.action_status_pub = rospy.Publisher(
        #     "/robodog/action_status", ActionStatus, queue_size=10
        # )

    # self.action_status = ActionStatus()

    def process_odom(self, data: Odometry):
        self.odom = data

    def process_scan(self, data: LaserScan):
        self.ranges = data.ranges

    def set_speed(self, linear_x: float = 0.0, angular_z: float = 0.0):
        speed = Twist()
        speed.linear.x = linear_x
        speed.angular.z = angular_z

        self.speed_pub.publish(speed)

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
        self.set_speed()

        if center == float("inf"):
            return Result.FAILURE
        return Result.SUCCESS

    def go_to(self, color: str) -> Result:
        if self.turn_to(color) is Result.FAILURE:
            print(f"Could not find {color}")
            return Result.FAILURE

        distance = self.ranges[0]
        center = self.image_processor.get_center_for_color(color)

        rate = rospy.Rate(10)
        speed = 0
        max_speed = 1
        while distance > 0.4 or abs(center) > 10:
            center = self.image_processor.get_center_for_color(color)
            distance = self.ranges[0]

            linear = min(speed + 0.05, max_speed, distance * 0.1)
            angular = -center * 0.01
            self.set_speed(linear_x=linear, angular_z=angular)
            rate.sleep()

        return Result.SUCCESS

    def follow(self, color: str) -> Result:
        if self.turn_to(color) is Result.FAILURE:
            print(f"Could not find {color}")
            return Result.FAILURE

        while not self.ranges:
            pass

        distance = self.ranges[0]
        center = self.image_processor.get_center_for_color(color)

        rate = rospy.Rate(10)
        speed = 0
        max_speed = 1
        stopped_time = 0
        current_time = time.time()
        while distance > 0.3 or abs(center) > 10 or stopped_time < 5:
            center = self.image_processor.get_center_for_color(color)
            distance = self.ranges[0]

            if abs(center) <= 10:
                angular = 0
            elif center == float("inf"):
                angular = 0.5
            else:
                angular = min(-center * 0.01, 0.5)

            if distance <= 0.3 or center == float("inf"):
                linear = 0
            else:
                linear = min(speed + 0.05, max_speed, distance * 0.1)
            self.set_speed(linear_x=linear, angular_z=angular)

            if linear == 0 and angular == 0:
                stopped_time = time.time() - current_time
            else:
                current_time = time.time()
            rate.sleep()

        return Result.SUCCESS

    def pick_up_dumbbell(self, color: str) -> Result:
        """
        Goes to dumbbell and picks it up

        Parameters:
            color: string of desired color to be picked up
        Returns:
            Result enum type of FAILURE or SUCCESS
        """

        if self.go_to(color) is Result.FAILURE:
            return Result.FAILURE

        rate = rospy.Rate(10)

        # open grip to get dumbbell
        self.arm_manipulator.open_grip()
        
        while self.ranges[0] > 0.2:
            rate.sleep()
            continue
        

        self.arm_manipulator.close_grip()
        
        self.set_speed()
        self.arm_manipulator.lift_dumbbell()

        # check to see if succesfully lifted dumbbell
        if min(self.ranges) > 0.3:
            return Result.SUCCESS
        else:
            return Result.FAILURE

    def fetch(self, color: str) -> Result:
        """
        Goes to dumbbell, picks it up, returns to person

        Parameters:
            color: string of desired color to be picked up
        Returns:
            Result enum type of FAILURE or SUCCESS
        """

        # pick up dumbbell
        if self.pick_up_dumbbell(color) is Result.FAILURE:
            return Result.FAILURE

        # go to person
        if self.go_to("yellow") is Result.FAILURE:
            self.place_dumbbell()
            return Result.FAILURE

        # place dumbbell
        self.place_dumbbell()

        return Result.SUCCESS

    def come(self) -> Result:
        """make bot come to person"""
        if self.go_to("yellow") is Result.FAILURE:
            return Result.FAILURE
        
        return Result.SUCCESS

    def find(self, color: str) -> Result:
        """ finds dumbbell of specified color and goes to it """
        if self.go_to(color) is Result.FAILURE:
            return Result.FAILURE
        
        return Result.SUCCESS
    
    def shake(self) -> Result:
        """make bot perform shake"""
        self.arm_manipulator.shake()
        # self.action_status.complete = True
        # self.action_status_pub.publish(self.action_status)
        return Result.SUCCESS

    def place_dumbbell(self) -> Result:
        """puts dumbbell on ground & move away"""
        self.set_speed()
        # place dumbbell on ground
        self.arm_manipulator.close_grip()
        self.arm_manipulator.reset_arm_position()

        # reverse away from dumbbell
        self.set_speed(linear_x=-0.2)
        rospy.sleep(2)

        # stop
        self.set_speed()

        return Result.SUCCESS

    def spin(self) -> Result:
        """ robot spins in a circle """
        turn_distance = 0
        speed = 0.5
        rate = rospy.Rate(10)

        # make bot spin 360 deg
        while abs(turn_distance) < np.pi * 2:
            turn_distance += speed / 10
            self.set_speed(angular_z=speed)
            rate.sleep()

        # stop spinning
        self.set_speed()
        # self.action_status.complete = True
        # self.action_status_pub.publish(self.action_status)
        return Result.SUCCESS

    def run(self):
        self.fetch("green")
        self.fetch("red")
        self.spin()


class ArmManipulator:
    def __init__(self):
        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander(
            "gripper"
        )

        # Position the arm
        self.reset_arm_position()

    def reset_arm_position(self):
        arm_joint_goal = [0.0, 0.4, 0.5, -0.9]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.open_grip()

    def open_grip(self):
        gripper_joint_goal = [.015,.015]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

    def close_grip(self):
        gripper_joint_goal = [.01,.01]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

    def sad_emote(self):
        """complete sad emote for bot"""
        arm_joint_goal = [1, 0.5, -0.5, 1.0]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.reset_arm_position()

    def happy_emote(self):
        """happy emote for bot"""
        arm_joint_goal = [0, -0.5, -0.3, -0.15]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.reset_arm_position()

    def shake(self):
        """complete shake action for bot"""
        arm_joint_goal = [0, 0.5, -0.2, -0.2]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        arm_joint_goal = [0, 0.5, -0.6, -0.2]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.reset_arm_position()

    def pick_up_position(self):
        arm_joint_goal = [0, 0.4, 0.5, -0.9]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        self.open_grip()

    def lift_dumbbell(self):
        """ move arm to lifted position"""
        arm_joint_goal = [
            0,
            math.radians(0.0),
            math.radians(-20),
            math.radians(-20),
        ]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()


if __name__ == "__main__":
    RobotControl().run()
