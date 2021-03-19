#!/usr/bin/env python3

import rospy
import time
from robodog.msg import (
    UserCommand, 
    Reward, 
    ActionStatus, 
    Action, 
    LearningMatrix, 
    LearningMatrixRow
)
import random


class PhantomDogMovement(object):

    def __init__(self):
        print("Phantom Initializing...")
        self.initialized = False

        # initialize this node
        rospy.init_node('dog_phantom_movement')

        # ROS publisher to send a generated user command to the learning algorithm
        self.command_pub = rospy.Publisher('/robodog/user_cmd', UserCommand, queue_size=10)

        # ROS publisher and subscriber to send and get the action that the robodog needs to execute
        self.action_pub = rospy.Publisher('robodog/action', Action, queue_size=10)
        rospy.Subscriber('robodog/action', Action, self.get_action)
        self.action_sent = None

        # ROS subscriber and publisher to get and send the current action status of the program
        rospy.Subscriber('/robodog/action_status', ActionStatus, self.get_action_status)
        self.action_status_pub = rospy.Publisher('/robodog/action_status', ActionStatus, queue_size=10)
        self.action_status ="Idle"

        # ROS publisher to send the reward for matrix update
        self.reward_pub = rospy.Publisher('/robodog/action_reward', Reward, queue_size=10)

        # ROS subscriber to get the current learning matrix
        rospy.Subscriber('/robodog/learning_matrix', LearningMatrix, self.get_matrix)
        self.learning_matrix =None

        self.action_iteration = 0
        
        # Command set
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

        self.robot_command_to_take = None
        # Tracks the number of tricks that have converged
        self.action_num = 0
        self.initialized = True
        print("Phantom initialization complete")
        print("WARNING: Whether the matrix converges depends on each individual trick run. Runs can take anywhere from 50 to 500 iterations to converge. ")

        self.execute_robot_action()
        
    def execute_robot_action(self):
        # Robot action iterator
        if not self.initialized:
            return 
        time.sleep(0.5)
        
        if self.action_num < 10:
            # Get and send robot action
            self.robot_command_to_take = self.select_command()
            command = UserCommand()
            command.command = self.robot_command_to_take
            self.command_pub.publish(command)
            self.action_status_pub.publish(ActionStatus(status ="Complete"))
            # Wait for publisher to update
            while self.action_status != "Complete":
                pass
            
            # Determine reward
            reward = self.all_or_nothing_rewarding(
                self.robot_command_to_take, self.action_sent)
            self.reward_pub.publish(reward)
            self.action_status_pub.publish(ActionStatus(status = "Idle"))

            self.action_iteration += 1
            
            # Start checking convergence after 20 iterations
            if self.action_iteration > 20:
                self.check_convergence()
            
            self.execute_robot_action()
    
     def all_or_nothing_rewarding(self, actual_command, processed_commad):
        # Trains the model without partial rewarding, i.e. either correct (10) or nothing (0)
        if self.action_num == processed_commad:
            return 10
        return 0
    
    def check_convergence(self):
        # Convergence checker
        # If a value has a probability > 99%, it is considered as converged
        val = self.commands[self.robot_command_to_take]
        matrix_val_row = list(self.learning_matrix[val].matrix_row[:])[:]
        NewMax = 1
        NewMin = 0
        OldMin = min(matrix_val_row) 
        OldMax = max(matrix_val_row)
        OldRange =  OldMax- OldMin  
        NewRange = NewMax - NewMin
        
        if OldRange == 0:
            return
        probability_sum = 0.0

        # Normalize matrix values
        for p in range(len(matrix_val_row)):
            matrix_val_row[p] = (
                ((matrix_val_row[p] - OldMin) * NewRange) / OldRange) + NewMin
            probability_sum += matrix_val_row[p]

        for p in range(len(matrix_val_row)):
            matrix_val_row[p] = matrix_val_row[p] / probability_sum

        # Probability greater than 99%
        if 1.0 - matrix_val_row[val] < .01:
            print(
                "Yay! RoboDog mastered " + self.robot_command_to_take + " after " + 
                str(self.action_iteration) + " iterations!")
            self.action_num += 1
            self.action_iteration = 0

            # Check if all tricks have converged
            if self.action_num == 10:
                print(self.learning_matrix)
                rospy.signal_shutdown("RoboDog mastered all tricks!! Great training!")



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
