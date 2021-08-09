#!/usr/bin/env python

"""
    File:
        set_goal.py
    Description:
        Set desired goal position 
    Author:
        Bruno <@usp.br>
"""
# ros libs
import rospy
import actionlib

#msg ros libs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID, GoalStatusArray
from nav_msgs.msg import Odometry

#internal libs
from motor import MotorControl

class SetGoal:
    def __init__(self):
        self.goal = MoveBaseGoal()
    

    def __str__(self):
        return 'Call Go function passing position coordinates and orientation coordinates as parameters'

    # Callbacks definition

    def initialise(self):
        self.navclient = actionlib.simple_action_client.SimpleActionClient("/move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        # Wait 60 seconds for the action server to become available
        self.navclient.wait_for_server(rospy.Duration(20))
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting move base goals smoother")

    def active_cb(self):
        rospy.loginfo("Goal pose being processed")

    def feedback_cb(self, feedback):
        rospy.loginfo("Current location: "+str(feedback.base_position.pose))

    def done_cb(self, status, result):
        if status == 3:
            rospy.loginfo("Goal reached")
        if status == 2 or status == 8:
            rospy.loginfo("Goal cancelled")
        if status == 4:
            rospy.loginfo("Goal aborted")

    def stop(self):
        self.navclient.cancel_goal()

    def go(self, des_pos_x, des_pos_y, des_pos_z, des_ori_x, des_ori_y, des_ori_z, des_ori_w):
        '''
        Description: 
        Get the robot to the desired goal
        Args:
        desired position for x, y and z
        desired orientation for x, y, z and w as quaternation in radians
        '''

        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()

        self.goal.target_pose.pose.position.x = des_pos_x
        self.goal.target_pose.pose.position.y = des_pos_y
        self.goal.target_pose.pose.position.z = des_pos_z
        self.goal.target_pose.pose.orientation.x = des_ori_x
        self.goal.target_pose.pose.orientation.y = des_ori_y
        self.goal.target_pose.pose.orientation.z = des_ori_z #0.662
        self.goal.target_pose.pose.orientation.w = des_ori_w #0.750

        self.navclient.send_goal(self.goal, self.done_cb, self.active_cb, self.feedback_cb) 

        rospy.loginfo (self.navclient.get_result())

        