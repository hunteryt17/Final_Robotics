#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveDBToBlock

import time

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from robodog.msg import UserCommand, Reward, ActionStatus, Action, LearningMatrix, LearningMatrixRow
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

        self.total_iteration = 0
        self.action_iteration = 0
        self.per_action_iteration = {
            "fetch red": 0,
            "fetch blue": 0,
            "fetch green": 0,
            "find red": 0,
            "find blue": 0,
            "find green": 0,
            "come": 0,
            "follow": 0,
            "shake": 0,
            "roll": 0,
        }
        self.robot_command_to_take = None


        self.commands ={
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

        self.action_num = 0
        self.initialized = True
        self.execute_robot_action()

    def check_convergence(self):
        val = self.commands[self.robot_command_to_take]
        matrix_val_row = list(self.learning_matrix[val].matrix_row[:])[:]
        print(matrix_val_row)
        NewMax = 1
        NewMin = 0
        OldMin = min(matrix_val_row) 
        OldMax = max(matrix_val_row)
        OldRange =  OldMax- OldMin  
        NewRange = NewMax -NewMin
        probability_sum = 0.0
        if OldRange == 0:
            return
        # Use Q Matrix to select new action
        for p in range(len(matrix_val_row)):
            matrix_val_row[p] = (((matrix_val_row[p] - OldMin) * NewRange) / OldRange) + NewMin
            probability_sum += matrix_val_row[p]

        for p in range(len(matrix_val_row)):
            matrix_val_row[p] = matrix_val_row[p] / probability_sum

        
        if 1.0 - matrix_val_row[val] < .01:
            print("Mastered Action: " + self.robot_command_to_take)
            print(self.learning_matrix)
            self.per_action_iteration[self.robot_command_to_take] = self.action_iteration
            self.action_num+= 1
            self.action_iteration = 0
            if self.action_num == 10:
                print(self.per_action_iteration)
                rospy.signal_shutdown("Dog Mastered: " + self.robot_command_to_take +"\n \n")



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
        return actions[self.action_num]
        
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
        print("Command given: " + str(commands[actual_command]) )
        print("Processed Command: " + str(processed_commad))
        if commands[actual_command] == processed_commad:
            return 10
        return 0

    def execute_robot_action(self):
        if not self.initialized:
            return 
        time.sleep(0.5)
        if self.action_num < 10:
            self.robot_command_to_take = self.select_command()
            command = UserCommand()
            command.command = self.robot_command_to_take
            self.command_pub.publish(command)
            self.action_status_pub.publish(ActionStatus(status ="Complete"))
            while self.action_status != "Complete":
                pass
            reward = self.all_or_nothing_rewarding(self.robot_command_to_take, self.action_sent)
            self.reward_pub.publish(reward)
        
            self.action_status_pub.publish(ActionStatus(status = "Idle"))
            self.total_iteration += 1
            self.action_iteration += 1
            print("Action Complete: " + str(self.action_iteration))
            if self.action_iteration > 5:
                self.check_convergence()
            # rospy.sleep(1)
            self.execute_robot_action()


    def get_action_status(self, data):
        self.action_status = data.status

    def get_matrix(self, data):
        self.learning_matrix = data.matrix

    def get_action(self, data):
        self.action_sent = data.action


    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__=="__main__":

    node = PhantomDogMovement()
    node.run()