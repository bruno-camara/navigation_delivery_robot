#!/usr/bin/env python

"""
    File:
        test_set_goal_example.py
    Description:
        test set_goal 
    Author:
        Bruno <@usp.br>
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class SetGoal:
    def __init__(self, des_x, des_y):
        self.des_x = des_x
        self.des_y = des_y

    def __str__(self):
        return 'Goal: ({x}, {y})' .format(x = self.des_x, y = self.des_y)

    # Callbacks definition

    def active_cb(self, extra):
        rospy.loginfo("Goal pose being processed")

    def feedback_cb(self, feedback):
        rospy.loginfo("Current location: "+str(feedback))

    def done_cb(self, status, result):
        if status == 3:
            rospy.loginfo("Goal reached")
        if status == 2 or status == 8:
            rospy.loginfo("Goal cancelled")
        if status == 4:
            rospy.loginfo("Goal aborted")

    def go(self):
        rospy.init_node('send_goal')

        navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        navclient.wait_for_server()

        # Example of navigation goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.des_x
        goal.target_pose.pose.position.y = self.des_y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.662
        goal.target_pose.pose.orientation.w = 0.750

        navclient.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        finished = navclient.wait_for_result()

        if not finished:
            rospy.logerr("Action server not available!")
        else:
            rospy.loginfo ( navclient.get_result())