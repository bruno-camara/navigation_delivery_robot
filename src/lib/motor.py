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
                self.rotation=Twist()
                self.topic_name=topic_name

                pass

        def _str_(self):
                """ Description:
                print Control Motor topic name"""
                return 'sensor topic name = {topic_name}'.format(topic_name = self.topic_name)
                pass

        def initialise(self):
                """ Description:
                initialise Motor """
                self.vel_msg=Twist()
                rospy.Subscriber(self.topic_name, Twist, self._callback)
                self.motor_cmd=rospy.Publisher(self.topic_name, Twist, queue_size=1)
                rospy.loginfo('Initialised {topic_name} at {time}'.format(topic_name = self.topic_name,time = rospy.get_time()) )
                pass

        def _callback(self, data):
                """ Description:
                        callback to update values """
                self.velocity = data.linear
                self.rotation = data.angular
                pass

        def get_velocity(self):
                """ Description:
                        return velocity """
                return self.velocity
                pass

        def get_rotation(self):
                """ Description:
                        return rotation """
                return self.rotation
                pass

        def set_velocity(self,vel):
                """ Description:
                        set velocity """
                self.vel_msg.linear.x=vel
                self.velocity=vel
                self.motor_cmd.publish(self.vel_msg)
                pass

        def set_rotation(self,rot):
                """ Description:
                        set rotation """
                self.vel_msg.angular.z = rot
                self.rotation=rot
                self.motor_cmd.publish(self.vel_msg)
                pass

        pass
