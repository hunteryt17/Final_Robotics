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

        # print welcome and other information
        print("Welcome! Robodog awaits your command")
        print("Commands include: \n roll \n shake \n come \n follow \n"
        " find [color of dumbbell] \n fetch [color of dumbbell]" 
        "\nColor options: red, green, or blue"
        "\n\nOnce Robodog has completed your command, please give a reward on a scale of 1 to 10, 10 being amazing")
        print("To exit enter \'quit\' as a command")
        
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
                print("------------------------------------")
                self.command.command = input("What is your command? \n")
                # quit
                if self.command.command == "quit":
                    rospy.signal_shutdown("Goodbye")
                
                # check if command is valid
                # user inputed nothing
                elif not self.command.command:
                    print("Please type a command")

                # invalid command
                elif self.command.command not in cmds:
                    print("Invalid command, please try again")
               
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
                        
                    except ValueError:
                        print("Invalid reward, please input a number")
                        continue
                    
                    if self.reward.reward in range(1,11):
                        break
                    else:
                        print("Please enter a reward on a scale of 1 to 10")
                        
                
                print("Reward received")
                self.reward_pub.publish(self.reward)
                self.busy = False
                self.action_complete = False

        


if __name__=="__main__":
    node = UserInterface()
    node.run()
    
    
    
        