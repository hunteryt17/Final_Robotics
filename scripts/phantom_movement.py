#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveDBToBlock

import time

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from robodog.msg import UserCommand, Reward, ActionStatus, Action, LearningMatrix
import random


class PhantomDogMovement(object):

    def __init__(self):
        self.initialized = False
        # initialize this node
        rospy.init_node('dog_phantom_movement')

        # rospy.Subscriber('/robodog/user_cmd', UserCommand, self.get_command)
        self.command_pub = rospy.Publisher('/robodog/user_cmd', UserCommand, queue_size=10)

        # rospy.Subscriber('/robodog/action_reward', Reward, self.get_reward)
        self.action_pub = rospy.Publisher('robodog/action', Action, queue_size=10)
        rospy.Subscriber('robodog/action', Action, self.get_action)
        self.action_sent = None
        rospy.Subscriber('/robodog/action_status', ActionStatus, self.get_action_status)
        self.action_status_pub = rospy.Publisher('/robodog/action_status', ActionStatus, queue_size=10)
        self.action_status ="Idle"

        self.reward_pub = rospy.Publisher('/robodog/action_reward', Reward, queue_size=10)

        rospy.Subscriber('/robodog/learning_matrix', LearningMatrix, self.get_matrix)
        self.learning_matrix =None
        # information about the robot action to take
        self.dog_action_queue = []

        self.iteration = 1

        # numbered block model names
        self.actions_seq = [
            "fetch red",
            "fetch red",
            # "fetch red",
            # "fetch red",
            # "fetch red",
            # "fetch red",
            # "fetch red",
            # "fetch red",
            # "fetch red",
            # "fetch red",
            # "fetch red"
            # "fetch blue",
            # "fetch green",
            # "find red",
            # "find blue",
            # "find green",
            # "come",
            # "follow",
            # "shake",
            # "roll"
        ]

        self.initialized = True
        self.execute_robot_action()

    def select_command(self):
        
        actions = ["fetch red", 
            "fetch blue", 
            "fetch green",
            "find red",
            "find blue",
            "find green",
            "come",
            "follow",
            "shake",
            "roll"
        ]
        return random.choice(actions)
        # return "fetch red"
        
    def all_or_nothing_rewarding(self, actual_command, processed_commad):
        commands = {
            "fetch red": 0,
            "fetch blue": 1,
            "fetch green": 2,
            "find red": 3,
            "find blue": 4,
            "find green": 5,
            "come": 6,
            "follow": 7,
            "shake": 8,
            "roll": 9,
        }
        print("Commands does it match 1: " + str(commands[actual_command]) )
        print("Processed Command: " + str(processed_commad))
        if commands[actual_command] == processed_commad:
            return 10
        return 0

    def execute_robot_action(self):
        if not self.initialized:
            return 
        time.sleep(0.5)

        # if (len(self.actions_seq) > 0):

        robot_command_to_take = self.select_command()

        print(robot_command_to_take)
        command = UserCommand()
        command.command = robot_command_to_take
        self.command_pub.publish(command)
        self.action_status_pub.publish(ActionStatus(status ="Complete"))
        while self.action_status != "Complete":
            # print("waiting for action completion")
            pass
        # Give arbitary reward
        # curr_matrix = self.learning_matrix
        reward = self.all_or_nothing_rewarding(robot_command_to_take, self.action_sent)
        self.reward_pub.publish(reward)
     
        print("New matrix \n")
        print(self.learning_matrix)

        # self.actions_seq.pop(0)
        self.action_status_pub.publish(ActionStatus(status = "Idle"))
        self.iteration += 1
        print("Action Complete: " + str(self.iteration))
        # rospy.sleep(1)
        self.execute_robot_action()


    def get_action_status(self, data):
        self.action_status = data.status

    def get_matrix(self, data):
        self.learning_matrix = data.matrix

    def get_action(self, data):
        self.action_sent = data.action


    def run(self):
        rospy.spin()

if __name__=="__main__":

    node = PhantomDogMovement()
    node.run()