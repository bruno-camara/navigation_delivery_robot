#!/usr/bin/env python


import time
import rospy
from geometry_msgs.msg import Twist


calibration=100

class MotorControl():
    def _init_(self,topic_name):
        """ Description:
                create a new Motor object"""
        """ Args:
                topic_name(string):topic name to Motor Control """
        self.velocity=0
        self.rotation=0
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
        motor_cmd=rospy.Publisher(self.topic_name, Twist, queue_size=1)
        rospy.Subscriber(self.topic_name, Twist, self._callback)
        rospy.loginfo('Initialised {topic_name} at {time}'.format(topic_name = self.topic_name,time = rospy.get_time()) )
        pass

    def _callback(self, data):
        """ Description:
                callback to update values """
        self.velocity = data.linear.x*calibration
        self.rotation = data.angular.z*calibration
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
        self.velocity=vel
        motor_cmd.publish(self)
        pass
    
    def set_rotation(self,rot):
        """ Description:
                set rotation """
        self.rotation=rot
        motor_cmd.publish(self)
        pass

    pass