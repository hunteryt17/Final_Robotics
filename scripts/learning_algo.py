#!/usr/bin/env python3
import numpy as np
import rospy
from numpy.random import random_sample
from robodog.msg import (
    Action,
    ActionStatus,
    LearningMatrix,
    LearningMatrixRow,
    Reward,
    UserCommand,
)
import random


def draw_random_action(choices, probabilities):
    # Greedy epsilon
    epsilon = [0.9, 0.1]
    choice = np.random.choice([True, False], p=epsilon)

    # Determine if the action chould be randomly calculated
    if not choice:
        return np.random.choice(10)
    
    # Normalize the quality matrix to positive value range
    NewMax = 1
    NewMin = 0
    OldMin = min(probabilities) 
    OldMax = max(probabilities)
    OldRange =  OldMax- OldMin  
    NewRange = NewMax -NewMin

    # Check if newly initialized
    if sum(probabilities) == 0 or OldRange == 0:
        return random.choice(choices)

    probability_sum = 0.0
   
    for p in range(len(probabilities)):
        probabilities[p] = (((probabilities[p] - OldMin) * NewRange) / OldRange) + NewMin
        probability_sum += probabilities[p]

    for p in range(len(probabilities)):
        probabilities[p] = probabilities[p] / probability_sum


    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    inds = values[np.digitize(random_sample(1), bins)]
    return choices[int(inds[0])]


class LearningAlgo(object):
    def __init__(self):
        print("Initializing Learning Algo...")
        self.initialized = False

        rospy.init_node("learning_algo")

        # ROS subscriber to user_cmd to get the input dog command 
        rospy.Subscriber("/robodog/user_cmd", UserCommand, self.get_user_command)

        # ROS subscriber to accept the rewards awarded to the dog after action completion
        rospy.Subscriber("/robodog/action_reward", Reward, self.get_reward)

        # ROS publisher to send command for action controller for robodog to execute
        self.action_pub = rospy.Publisher(
            "robodog/action", Action, queue_size=10
        )

        # ROS publisher to update the Q-Matrix for the learning
        self.matrix_pub = rospy.Publisher(
            "robodog/learning_matrix", LearningMatrix, queue_size=10
        )

        # Initialize Q Matrix
        self.q_matrix = LearningMatrix()
        self.initialize_matrix()

        
        self.user_command = None
        self.selected_action = None
        self.reward = None
        self.received_action = None

        self.initialized = True
        print("Initialization Complete")


    def initialize_matrix(self):
        for _ in range(10):
            x = LearningMatrixRow()
            for _ in range(10):
                x.matrix_row.append(.1)
            self.q_matrix.matrix.append(x)
        
        # To run trained model uncomment section below
        # self.q_matrix.matrix = (
        #     LearningMatrixRow([1,0,0,0,0,0,0,0,0,0]),
        #     LearningMatrixRow([0,1,0,0,0,0,0,0,0,0]),
        #     LearningMatrixRow([0,0,1,0,0,0,0,0,0,0]),
        #     LearningMatrixRow([0,0,0,1,0,0,0,0,0,0]),
        #     LearningMatrixRow([0,0,0,0,1,0,0,0,0,0]),
        #     LearningMatrixRow([0,0,0,0,0,1,0,0,0,0]),
        #     LearningMatrixRow([0,0,0,0,0,0,1,0,0,0]),
        #     LearningMatrixRow([0,0,0,0,0,0,0,1,0,0]),
        #     LearningMatrixRow([0,0,0,0,0,0,0,0,1,0]),
        #     LearningMatrixRow([0,0,0,0,0,0,0,0,0,1])
        # )
        self.matrix_pub.publish(self.q_matrix)

    def get_reward(self, data):
        if not self.initialized:
            return

        self.reward = data.reward
        self.update_q_matrix()

    def update_q_matrix(self):
        if not self.initialized:
            return

        alpha = 1
        gamma = 0.01

        current_val = self.q_matrix.matrix[self.received_action].matrix_row[
            self.selected_action
        ]
        max_action = max(
            self.q_matrix.matrix[self.received_action].matrix_row
        )

        # Determines if the roboDog should receive a negative reward
        if self.reward == 0:
            reward = -10
        else:
            reward = self.reward

        change = alpha * (
            (reward / 10.0) + gamma * max_action - current_val
        )
        self.q_matrix.matrix[self.received_action].matrix_row[
            self.selected_action] = current_val + change

        self.matrix_pub.publish(self.q_matrix)

    def get_user_command(self, data):
        if not self.initialized:
            return
        self.user_command = data.command

        # Begin action sequence once user command is received
        self.select_action()

    def process_command(self):
        if not self.initialized:
            return
        # Converts string command to integer value for matrix
        command = self.user_command
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
        action = commands[command]
        return action

    def select_action(self):
        if not self.initialized:
            return

        # Converts command to numeric
        self.received_action = self.process_command()

        # Select an action using the matrix values for the received action
        actions = list(range(10))
        probabilities = self.q_matrix.matrix[self.received_action].matrix_row[:]
        self.selected_action = draw_random_action(
            actions, probabilities
        )

        # Publish action for robot execution
        action = Action()
        action.action = self.selected_action
        self.action_pub.publish(action)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    node = LearningAlgo()
    node.run()
