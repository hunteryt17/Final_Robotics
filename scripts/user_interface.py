#!/usr/bin/env python3

'''
variables:
- self.busy: boolean to indicate whether the program is ready to take a command
- self.action_complete: boolean to indicate if action is complete; subscribes
  to robodog/action_status which shld be published to by action execution node

1. when rosrun robodog user_interface.py is executed, Welcome, etc. is printed
in the terminal
2. self.busy and self.action_complete are both initialized to False so "What 
   is your command?" prompt is printed to screen
3. checks if the user input command is valid:
    a. if invalid, error message is printed and loop continues without changing 
       self.busy or self.action_complete so command prompt printed again
    b. if valid, publish command to /robodog/user_cmd and set self.busy to True
4. once action execution code published action as complete, user is prompted
   to input a reward (integar)
5. reward is published to /robodog/action_reward and self.busy and
   self.action_complete are reset to False


'''

import rospy

from robodog.msg import UserCommand, Reward, ActionStatus

commands = ['roll','shake','come','follow','find','fetch']
colors = ['red','green','blue']

class UserInterface(object):
    def __init__(self):
        rospy.init_node('robodog_user_interface')
        print("Welcome! Robodog awaits your command")
        print("Commands include: \n roll \n shake \n come \n follow \n"
        " find [color of dumbbell] \n fetch [color of dumbbell]")
        
        self.command = UserCommand()
        self.reward = Reward()
        self.busy = False
        self.action_complete = False

        self.cmd_pub = rospy.Publisher('/robodog/user_cmd', UserCommand, queue_size=10)
        self.reward_pub = rospy.Publisher('/robodog/action_reward', Reward, queue_size=10)
        
        rospy.Subscriber('/robodog/action_status', ActionStatus, self.get_action_status)

        # self.process_input()

    def get_action_status(self, status_msg):
        self.action_complete = status_msg.complete
        
    def run(self):
        while not rospy.is_shutdown():
            if not self.busy and not self.action_complete:
                self.command.command = input("What is your command? \n")
                # check if command is valid
                cmd_list = self.command.command.split()
                if not cmd_list:
                    print("Please type a command")
                elif cmd_list[0] not in commands:
                    print("Could not process \"" + self.command.command + "\", please try again")
                elif cmd_list[0] in ['find','fetch'] and (len(cmd_list) != 2 \
                    or cmd_list[1] not in colors):
                        print("Invalid color for \"" + cmd_list[0] + "\", please try again")
                else:   
                    print("Executing command: " + self.command.command)
                    self.cmd_pub.publish(self.command)
                    self.busy = True
            elif self.action_complete:
                print("Action complete")
                while True: 
                    try:
                        self.reward.reward = int(input("How did Robodog do? \n"))
                        break
                    except ValueError:
                        print("Invalid reward, please input a number")
                print("Reward received")
                self.reward_pub.publish(self.reward)
                self.busy = False
                self.action_complete = False

        

    # def run(self):
    #     while not rospy.is_shutdown():
    #         rospy.spin()


if __name__=="__main__":
    node = UserInterface()
    node.run()
    
    
    
        