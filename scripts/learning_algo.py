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


def draw_random_action(choices, probabilities):
    epsilon = [0.9, 0.1]
    choice = np.random.choice([True, False], p=epsilon)
    # randomly select action
    if not choice:
        return np.random.choice(10)

    probability_sum = 0.0
    # Use Q Matrix to select new action
    for p in range(len(probabilities)):
        probabilities[p] += 1
        probability_sum += probabilities[p]

    for p in range(len(probabilities)):
        probabilities[p] = probabilities[p] / probability_sum

    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    print(values)
    print(probs)
    print(bins)
    inds = values[np.digitize(random_sample(1), bins)]
    return choices[int(inds[0])]


class LearningAlgo(object):
    def __init__(self):
        self.initialized = False
        print("Initializing Learning Algo...")

        rospy.init_node("learning_algo")
        rospy.Subscriber("/robodog/user_cmd", UserCommand, self.get_command)
        rospy.Subscriber("/robodog/action_reward", Reward, self.get_reward)
        self.action_pub = rospy.Publisher(
            "robodog/action", Action, queue_size=10
        )

        self.matrix_pub = rospy.Publisher(
            "robodog/learning_matrix", LearningMatrix, queue_size=10
        )
        # rospy.Subscriber(
        #     "/robodog/action_status", ActionStatus, self.get_status
        # )
        self.action_status = "Idle"
        self.command = None
        self.selected_action = None
        self.reward = None
        self.processed_action = None
        self.q_matrix = LearningMatrix()
        self.initialize_matrix()

        self.initialized = True

    def initialize_matrix(self):
        for _ in range(10):
            x = LearningMatrixRow()
            for _ in range(10):
                x.matrix_row.append(0)
            self.q_matrix.matrix.append(x)
        # self.q_matrix.matrix[0] = LearningMatrixRow([1,0,0,0,0,0,0,0,0,0])
        self.matrix_pub.publish(self.q_matrix)

    # def get_status(self, data):
    #     self.action_status = data.status

    def get_reward(self, data):
        if not self.initialized:
            return
        self.reward = data.reward
        self.update_action_probs()

    def update_action_probs(self):

        if not self.initialized:
            return
        alpha = 1
        gamma = 0.5

        current_val = self.q_matrix.matrix[self.processed_action].matrix_row[
            self.selected_action
        ]
        max_action = max(
            self.q_matrix.matrix[self.processed_action].matrix_row
        )
        change = alpha * (
            (self.reward / 10.0) + gamma * max_action - current_val
        )
        self.q_matrix.matrix[self.processed_action].matrix_row[
            self.selected_action] = current_val + change
        # print(self.q_matrix)
        self.matrix_pub.publish(self.q_matrix)
        # print("New matrix")
        # print(self.q_matrix)

    def get_command(self, data):
        if not self.initialized:
            return
        self.command = data.command

        self.select_action()

    def process_command(self):
        if not self.initialized:
            return
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
            "roll": 9,
        }
        command_lowered = command.lower()
        action = None
        for comm in commands.keys():
            comm_parsed = comm.split()
            if len(comm_parsed) == 1:
                if comm_parsed[0] in command_lowered:
                    action = commands[comm]
            else:
                if (
                    comm_parsed[0] in command_lowered
                    and comm_parsed[1] in command_lowered
                ):
                    action = commands[comm]
        return action

    def select_action(self):
        if not self.initialized:
            return

        # Subscribe to topic that sends the processed command and convert
        # command to numeric
        self.processed_action = self.process_command()
        print("Processed Command: " + str(self.processed_action))
        # Select an action using the action probability matrix
        actions = list(range(10))
        probabilities = self.q_matrix.matrix[self.processed_action].matrix_row[:]
        # print(probabilities)
        self.selected_action = draw_random_action(
            actions, probabilities
        )
        # print(probabilities)
        # print( self.q_matrix.matrix[self.processed_action].matrix_row)
        # Publish the action
        action = Action()
        action.action = self.selected_action
        self.action_pub.publish(action)
        print("Selected Action: " + str(self.selected_action))
        print("action complete\n")

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    node = LearningAlgo()
    node.run()
