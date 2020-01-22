#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseStamped
import actionlib


class ResendGoal:
    def __init__(self):
        self.current_goal = MoveBaseGoal()
        self.NO_MSG = True
        self.sub = rospy.Subscriber('/move_base/current_goal', PoseStamped, callback=self.move_callback, queue_size=1)
        self.resend_server = rospy.Service('resend_goal', Empty, self.resend_goal)
        self.movebase_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.movebase_client.wait_for_server()
        rospy.loginfo('connected to MoveBase')

    def move_callback(self, msg):
        self.current_goal.target_pose = msg
        self.current_goal.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo('goal received')
        self.NO_MSG = False

    def resend_goal(self, req):
        if self.NO_MSG:
            self.current_goal.target_pose.pose.orientation.w = 1
            self.current_goal.target_pose.pose.position.x = 0.5
            self.current_goal.target_pose.header.stamp = rospy.Time.now()
        self.movebase_client.send_goal(self.current_goal)
        rospy.loginfo('resending goal')

        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('resend_goal')
    r = ResendGoal()
    rospy.spin()
