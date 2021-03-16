#!/usr/bin/env python3


import rospy

from robodog.msg import UserCommand, Reward, ActionStatus

# ActionStatus.status options are ['complete', 'failed', 'in progress', 'idle']


cmds = {
            0: "fetch red",
            1: "fetch blue",
            2: "fetch green",
            3: "find red",
            4: "find blue",
            5: "find green",
            6: "come",
            7: "follow",
            8: "shake",
            9: "roll",
        }
class UserInterface(object):
    def __init__(self):
        # initialize node
        rospy.init_node('robodog_user_interface')

        # print welcome and other information
        print("Welcome! Robodog awaits your command")
        print("Commands include: \n roll \n shake \n come \n follow \n"
        " find [color of dumbbell] \n fetch [color of dumbbell]" 
        "\nColor options: red, green, or blue"
        "\n\nOnce Robodog has completed your command, please give a reward on a scale of 0 to 10, 10 being amazing")
        print("To exit enter \'quit\' as a command")
        
        # initialize
        self.command = UserCommand()
        self.reward = Reward()
        self.counts = dict.fromkeys(cmds.values(), 0)

        # booleans to determine state of robodog 
        self.busy = False
        self.action_status = 'idle'
        self.action_completed = 0
        
        # set up publishers and subscribers
        self.cmd_pub = rospy.Publisher('/robodog/user_cmd', UserCommand, queue_size=10)
        self.reward_pub = rospy.Publisher('/robodog/action_reward', Reward, queue_size=10)
        
        rospy.Subscriber('/robodog/action_status', ActionStatus, self.get_action_status)

        
    def get_action_status(self, status_msg):
        self.action_status = status_msg.status
        self.action_completed = status_msg.action
        
    def run(self):
        while not rospy.is_shutdown():
            # robot is available to get command
            if not self.busy and self.action_status == 'idle':
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
                elif self.command.command not in cmds.values():
                    print("Invalid command, please try again")
               
                # valid command
                else:   
                    print("Executing command: " + self.command.command)
                    self.command.iteration_num = self.counts[self.command.command]
                    self.cmd_pub.publish(self.command)
                    self.busy = True
                    self.counts[self.command.command] += 1

            # robot completed action and needs a reward
            elif self.action_status == 'complete' or self.action_status == 'failed':
                if self.action_status == 'failed':
                    print("Oops! Something went wrong...")
                    print("Still please give Robodog a reward")
                    print("Action attempted:  " + cmds[self.action_completed])
                else:
                    print("Action completed:  " + cmds[self.action_completed])

                # make sure user input is valid
                while True: 
                    try:
                        self.reward.reward = int(input("How did Robodog do? \n"))
                        
                    except ValueError:
                        print("Invalid reward, please input a number")
                        continue
                    
                    if self.reward.reward in range(0,11):
                        break
                    else:
                        print("Please enter a reward on a scale of 0 to 10")
                        
                print("Reward received")
                self.reward_pub.publish(self.reward)
                self.busy = False
                self.action_status = 'idle'

        


if __name__=="__main__":
    node = UserInterface()
    node.run()
    
    
    
        