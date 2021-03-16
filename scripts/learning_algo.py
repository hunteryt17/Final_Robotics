#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan
from q_learning_project.msg import QLearningReward
from std_msgs.msg import Header

from random import shuffle
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import time
import math
import random
import numpy as np
from robodog.msg import UserCommand, Reward, ActionStatus, Action, LearningMatrix
from numpy.random import random_sample 
from copy import deepcopy

# commands = ['roll','shake','come','follow','find','fetch']
# colors = ['red','green','blue']

def draw_random_action(choices, probabilities):
    """ Return a random action from the set choices with the specified probabilities
        choices: the values to sample from represented as a list
        probabilities: the probability of selecting each element in choices represented as a list
    """
    episilon = [0.9,0.1]
    random = np.random.choice([True, False], p= episilon)
    # randomly select action
    if not random:
        return np.random.choice(10)

    # Use Q Matrix to select new action
    for p in probabilities:
        p += 1
    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    print(values)
    print(probs)
    print(bins)
    inds = values[np.digitize(random_sample(10), bins)]
    samples = []
    for i in inds:
        samples.append(deepcopy(choices[int(i)]))
    action = samples[0]
    return action


class LearningAlgo(object):

    def __init__(self):
        self.initialized = False
        print("Initializing Learning Algo...")

        rospy.init_node('learning_algo') 
        rospy.Subscriber('/robodog/user_cmd', UserCommand, self.get_command)
        rospy.Subscriber('/robodog/action_reward', Reward, self.get_reward)
        self.action_pub = rospy.Publisher('robodog/action', Action, queue_size=10)
        self.matrix_pub = rospy.Publisher('robodog/learning_matrix', LearningMatrix, queue_size=10)
        rospy.Subscriber('/robodo/action_status', ActionStatus,self.get_status)
        self.action_status = "Idle"
        self.command = None
        self.selected_action = None
        self.reward = None
        self.processed_action = None

        self.action_matrix = [[] for x in range(10)]
        for i in range(10):
            self.action_matrix[i] = [1/10 for x in range(10)]
        self.matrix_pub.publish(self.action_matrix)

        self.initialized = True

    def get_status(self, data):
        self.action_status = data.status

    def get_reward(self, data):
        self.reward = data.reward 
        # print(self.action_status)
        if self.reward and self.action_status == "Complete":
            # print("Received Reward")
            # print(self.reward)
            self.update_action_probs()

    def update_action_probs(self):
        alpha = 1
        gamma = 0.5

        # get value
        current_val = self.action_matrix[self.processed_action][self.selected_action]
        
        # get max value of all actions for state2
        max_action = max(self.action_matrix[self.processed_action])

        # update q matrix for state1 & action_t
        self.action_matrix[self.processed_action][self.selected_action]  += \
            int(alpha * (self.reward/10.0 + gamma * max_action  - current_val))
        print("Updating Action Matrix")
        print(self.action_matrix)
        matrix = LearningMatrix()
        matrix.matrix = self.action_matrix
        self.matrix_pub.publish(matrix)
 

    def get_command(self, data):
        if not self.initialized:
            return
        self.command = data.command
        
        self.select_action()

    def process_command(self):
        command = self.command
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
            "roll": 9
        }
        command_lowered = command.lower()
        action = None
        for comm in commands.keys():
            comm_parsed = comm.split()
            if len(comm_parsed) == 1:
                if comm_parsed[0] in command_lowered:
                    action = commands[comm]
            else:
                if comm_parsed[0] in command_lowered and comm_parsed[1] in command_lowered:
                    action = commands[comm]
        return action


    def select_action(self):
        if not self.initialized:
            return 
        
        # Subscribe to topic that sends the processed command and convert command to numeric
        self.processed_action = self.process_command()
        print(self.processed_action)
        # Select an action using the action probability matrix
        actions = [i for i in range(10)]
        self.selected_action = draw_random_action(actions, self.action_matrix[self.processed_action])
        
        # Publish the action
        action = Action()
        action.action = self.selected_action
        self.action_pub.publish(action)
        print(self.selected_action)
        print("action complete\n")


    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__=="__main__":
    node = LearningAlgo()
    node.run()

