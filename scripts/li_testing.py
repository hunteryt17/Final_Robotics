#!/usr/bin/env python3

import rospy


from robodog.msg import UserCommand, Reward, ActionStatus



class LiTesting(object):
    def __init__(self):
        rospy.init_node('li_testing')
        
        self.command = UserCommand()
        self.reward = Reward()
        self.action_complete = False

        self.status_pub = rospy.Publisher('/robodog/action_status', ActionStatus, queue_size=10)

        rospy.Subscriber('/robodog/user_cmd', UserCommand, self.process_command)
        rospy.Subscriber('/robodog/action_reward', Reward, self.get_reward)        


    
    def process_command(self, cmd_msg):
        self.action_complete = False
        self.status_pub.publish(self.action_complete)
        self.command = cmd_msg
        # do action
        
        self.action_complete = True
        self.status_pub.publish(self.action_complete)
        

    def get_reward(self, reward_msg):
        self.reward = reward_msg    
    

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


if __name__=="__main__":
    node = LiTesting()
    node.run()