#! /usr/bin/env python3
import rospy
from robodog.msg import Action, ActionStatus
import actions


class RobotController:
    def __init__(self):
        rospy.init_node("robodog_controller")
        self.robot_actions = actions.RobotActions()

        self.num_action_map = {
            0: (lambda: self.robot_actions.go_to("red")),
            1: (lambda: self.robot_actions.go_to("blue")),
            2: (lambda: self.robot_actions.go_to("green")),
            3: (lambda: self.robot_actions.go_to("red")),
            4: (lambda: self.robot_actions.go_to("green")),
            5: (lambda: self.robot_actions.go_to("blue")),
            6: (lambda: self.robot_actions.go_to("yellow")),
            7: (lambda: self.robot_actions.follow("yellow")),
            8: (lambda: self.robot_actions.shake()),
            9: (lambda: self.robot_actions.spin()),
        }

        self.action_subscriber = rospy.Subscriber(
            "robodog/action", Action, self.process_action
        )

        self.action_status_publisher = rospy.Publisher(
            "robodog/action_status", ActionStatus, queue_size=10
        )

    def process_action(self, action: Action):
        rospy.loginfo(f"############\n###########\nEXECUTING {action.action}")
        result = self.num_action_map[action.action]()
        if result is actions.Result.FAILURE:
            self.action_status_publisher.publish(
                ActionStatus(status="complete", action=action.action)
            )

        else:
            self.action_status_publisher.publish(
                ActionStatus(status="failed", action=action.action)
            )

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    RobotController().run()
