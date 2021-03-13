#!/usr/bin/env python3

'''
variables:
- self.busy: boolean to indicate whether the program is ready to take a command
- self.action_complete: boolean to indicate if action is complete; subscribes
  to robodog/action_status which shld be published to by action execution node

'''

import rospy

from robodog.msg import UserCommand, Reward, ActionStatus

commands = ['roll','shake','come','follow','find','fetch']
colors = ['red','green','blue']
cmds = ['roll','shake','come','follow','find red','find green','find blue', \
    'fetch red','fetch green','fetch blue']

class UserInterface(object):
    def __init__(self):
        # initialize node
        rospy.init_node('robodog_user_interface')

        # print welcome and list of possible commands
        print("Welcome! Robodog awaits your command")
        print("Commands include: \n roll \n shake \n come \n follow \n"
        " find [color of dumbbell] \n fetch [color of dumbbell]" 
        " Color options: red, green, or blue"
        " Once Robodog has completed your command, please give a reward on a scale of 1 to 10, 10 being amazing")
        
        # initialize
        self.command = UserCommand()
        self.reward = Reward()
        self.counts = dict.fromkeys(cmds, 0)

        # booleans to determine state of robodog 
        self.busy = False
        self.action_complete = False
        
        # set up publishers and subscribers
        self.cmd_pub = rospy.Publisher('/robodog/user_cmd', UserCommand, queue_size=10)
        self.reward_pub = rospy.Publisher('/robodog/action_reward', Reward, queue_size=10)
        
        rospy.Subscriber('/robodog/action_status', ActionStatus, self.get_action_status)

        
    def get_action_status(self, status_msg):
        self.action_complete = status_msg.complete
        
    def run(self):
        while not rospy.is_shutdown():
            # robot is available to get command
            if not self.busy and not self.action_complete:
                self.command.command = input("What is your command? \n")
                
                # check if command is valid
                # cmd_list = self.command.command.split()

                # user inputed nothing
                if not cmd_list:
                    print("Please type a command")
                elif self.command.command not in cmds:
                    print("Invalid command, please try again")

                # invalid command
                # elif cmd_list[0] not in commands:
                #     print("Could not process \"" + self.command.command + "\", please try again")

                # # invalid find or fetch command
                # elif cmd_list[0] in ['find','fetch'] and (len(cmd_list) != 2 \
                #     or cmd_list[1] not in colors):
                #         print("Invalid color for \"" + cmd_list[0] + "\", please try again")
                
                # valid command
                else:   
                    print("Executing command: " + self.command.command)
                    self.command.iteration_num = self.counts[self.command.command]
                    self.cmd_pub.publish(self.command)
                    self.busy = True
                    self.counts[self.command.command] += 1

            # robot completed action and needs a reward
            elif self.action_complete:
                print("Action complete")

                # make sure user input is valid
                while True: 
                    try:
                        self.reward.reward = int(input("How did Robodog do? \n"))
                        break
                    except ValueError:
                        print("Invalid reward, please input a number")
                
                while True:
                    if self.reward.reward not in range(1,11):
                        print("Please enter a reward on a scale of 1 to 10")
                    else:
                        break

                print("Reward received")
                self.reward_pub.publish(self.reward)
                self.busy = False
                self.action_complete = False

        


if __name__=="__main__":
    node = UserInterface()
    node.run()
    
    
    
        