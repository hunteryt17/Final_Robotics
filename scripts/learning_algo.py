# import rospy

# from gazebo_msgs.msg import ModelState, ModelStates
# from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
# from sensor_msgs.msg import LaserScan
# from q_learning_project.msg import QLearningReward
# from std_msgs.msg import Header

# from random import shuffle
# from tf.transformations import quaternion_from_euler, euler_from_quaternion
# from q_learning_project.msg import RobotMoveDBToBlock, QMatrix, QLearningReward, QMatrixRow 

# import time
# import math
# import random
# import numpy as np
# from robodog.msg import UserCommand, Reward, ActionStatus
# from numpy.random import random_sample 

# commands = ['roll','shake','come','follow','find','fetch']
# colors = ['red','green','blue']

# def draw_random_action(choices, probabilities):
#     """ Return a random action from the set choices with the specified probabilities
#         choices: the values to sample from represented as a list
#         probabilities: the probability of selecting each element in choices represented as a list
#     """
#     values = np.array(range(len(choices)))
#     probs = np.array(probabilities)
#     bins = np.add.accumulate(probs)
#     inds = values[np.digitize(random_sample(1), bins)]
#     action = choices[int(inds[0])]
#     return action


# class LearningAlgo(object):

#     def __init__(self):
#         self.initialized = False

#         rospy.init_node('learning_algo') 


#         rospy.Subscriber('/robodog/user_cmd', UserCommand, self.get_command)
#         rospy.Subscriber('/robodog/action_reward', Reward, self.get_reward)
        
#         rospy.Subscriber('/robodog/action_status', ActionStatus, self.get_action_status)
#         self.action_pub = rospy.Publisher('robodog/action', Action, queue_size=10)
#         self.command = None
#         self.curr_action = None

#         self.action_matrix = [[] for x in range(10)]
#         for i in range(10):
#             self.action_matrix[i] = [0 for x in range(10)]
        
#         self.initialized = True

#     def get_reward(self, data):
#         self.get_reward = data.reward 

#     def update_action_probs(self):
#         alpha = 1
#         gamma = 0.5
#         eps = 10

#         # get value
#         current_val = self.action_matrix[self.command][self.curr_action]
        
#         # get max value of all actions for state2
#         max_action = max(self.action_matrix[self.command])

#         # update q matrix for state1 & action_t
#         self.action_matrix[self.command][self.curr_action]  +=
#             int(alpha * (self.reward.reward + gamma * max_action  - current_val))
        
#         # Normalize probabilities
#         prob_sum = 0.0
#         for prob in self.action_matrix[self.command]:
#             prob_sum += prob

#         for i in range(10):
#             self.action_matrix[self.command][i] =  self.action_matrix[self.command][i]/prob_sum

#     def get_command(self, data):
#         if not self.initialized:
#             return
#         self.command = data.command
#         return

#     def process_command(self):
#         command = self.command
#         commands = {
#             "fetch red": 0,
#             "fetch blue": 1,
#             "fetch green": 2,
#             "find red": 3,
#             "find blue": 4,
#             "find green": 5,
#             "come": 6,
#             "follow": 7,
#             "shake": 8,
#             "roll": 9
#         }
#         command_lowered = command.lower()
#         action = None
#         for comm in commands.keys():
#             comm_parsed = comm.split(", ")
#             if len(comm_parsed) == 1:
#                 if comm_parsed[0] in command_lowered:
#                     action = commands[comm]
#             else:
#                 if comm_parsed[0] in command_lowered and comm_parsed[1] in command_lowered:
#                     action = commands[comm]
#         return action


#     def select_action(self):
#         if not self.initialized:
#             return 
#         # Subscribe to topic that sends the processed command
#         # Convert command to numeric
#         given_action = self.process_command
#         # Select an action using the action probability matrix
#         actions = [i for i in range(10)]
#         self.curr_action = draw_random_action(self.action_matrix[given_action])
#         self.action_pub.publish(self.curr_action)

    
#     def initialized_action_probability_matrix(self):
        
#         # Populate State Space
#         state_space = []
#         for command in range(10):
#             for function in range(10):
#                     state_space.append((command, function))
        
#         # Set action probability
#         for s1 in state_space:
#             for s2 in state_space:
#                 self.action_matrix[s1][s2] = self.get_action_probability(s1,s2)
#                 return

#     def get_action_probability(self, s1, s2):
#         fetch_actions = [0,1,2]
#         find_actions = [3,4,5]
#         person_actions = [6,7]
#         in_place_actions = [8,9]

#         total_actions = float(len(fetch_actions) + len(find_actions) + len(person_actions) + len(in_place_actions))
#         prob = 1

#         if s1 in fetch_actions:
#             # Adds a slight boast to improve learning speeds
#             denominator = total_actions + (fetch_actions -1)(fetch_actions)
#             if s2 in fetch_actions:
#                 prob = fetch_actions/denominator
#             else:
#                 prob = 1/denominator
#         elif s1 in find_actions:
#             denominator = total_actions + (find_actions -1)(find_actions)
#             if s2 in find_actions:
#                 prob = find_actions/denominator
#             else:
#                 prob = 1/denominator
#         elif s1 in person_actions:
#             denominator = total_actions + (person_actions -1)(person_actions)
#             if s2 in person_actions:
#                 prob = person_actions/denominator
#             else:
#                 prob = 1/denominator
#         elif s1 in in_place_actions:
#             denominator = total_actions + (in_place_actions -1)(in_place_actions)
#             if s2 in in_place_actions:
#                 prob = find_actions/denominator
#             else:
#                 prob = 1/denominator  
#         return prob 



#     def run(self):
#         pass

# if __name__=="__main__":
#     node = LearningAlgo()
#     node.run()

