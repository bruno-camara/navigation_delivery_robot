#!/usr/bin/env python


import time
import rospy
from geometry_msgs.msg import Twist


calibration=100

class MotorControl():

        def __init__(self,topic_name):
            """ Description:
            create a new Motor object"""
            """ Args:
            topic_name(string):topic name to Motor Control """
            self.velocity=Twist()
            self.topic_name=topic_name

            pass

        def _str_(self):
            """ Description:
            print Control Motor topic name"""
            return 'sensor topic name = {topic_name}'.format(topic_name = self.topic_name)

        def initialise(self):
            """Initialisation method. Sets ROS subscribers and publishers
            """
            self.vel_msg=Twist()
            rospy.Subscriber(self.topic_name, Twist, self._callback)
            self.motor_cmd=rospy.Publisher(self.topic_name, Twist, queue_size=1)
            rospy.loginfo('Initialised {topic_name} at {time}'.format(topic_name = self.topic_name,time = rospy.get_time()) )
            pass

        def _callback(self, data):
            """Callback function to ROS subscriber

            Args:
                data (Twist()): linear and angular velocity recieved
            """
            self.velocity.linear = data.linear
            self.velocity.angular = data.angular
            #self.rotation = data.angular
            pass

        def get_velocity(self):
            """Returns linear velocity

            Returns:
                Twist().linear: x, y and z linear velocity
            """
            return self.velocity.linear
            pass

        def get_rotation(self):
            """Returns angular velocity

            Returns:
                Twist().angular: x, y and z angular velocity
            """
            return self.velocity.angular
                #return self.rotation

        def set_velocity(self,vel):
            """Set x linear velocity

            Args:
                vel (float): linear velocity to be set
            """
            self.vel_msg.linear.x=vel
            self.velocity.linear.x=vel
            self.motor_cmd.publish(self.vel_msg)
            pass

        def set_rotation(self,rot):
            """Set z angular velocity

            Args:
                rot (float): angular velocity to be set
            """
            self.vel_msg.angular.z = rot
            #self.rotation=rot
            self.velocity.angular.z = rot
            self.motor_cmd.publish(self.vel_msg)
            pass

        pass
