#! /usr/bin/env python

import rospy
from dobot.srv import *
from move_base_msgs.msg import MoveBaseGoal

if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)
    rospy.spin()
