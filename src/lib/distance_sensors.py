#/usr/bin/env python
# coding=utf-8
"""
    File:
        distance_sensor.py
    Description:
        distance_sensor object
    Author:
        Vanderson Santos <vanderson.santos@usp.br>
"""

import rospy
import math
from sensor_msgs.msg import Range

class DistanceSensor:
    def __init__(self, topic_name, angle_py=[0.0, 0.0], position=[0.0,0.0,0.0]):
        """ Description:
                create a new Distance Sensor object"""
        """ Args:
                topic_name(string):topic name to distance sensor
                angle_py(list): direction at which sensor is facing
                    angle_py[0]: vertical angle
                    angle_py[1]: horisontal angle
                position(list): relative position to origin(x,y,z) """
        self.distance = 0
        self.max_distance = 0
        self.min_distance = 0
        self.position = list(position[:])
        self.angle = list(angle_py[:])
        self.topic_name = topic_name
        pass

    def __str__(self):
        """ Description:
                print distance sensor topic name"""
        return 'sensor topic name = {topic_name}'.format(topic_name = self.topic_name)
        pass

    def initialise(self):
        """ Description:
                initialise distance sensor """
        rospy.Subscriber(self.topic_name, Range, self._callback)
        rospy.loginfo('Initialised {topic_name} at {time}'.format(topic_name = self.topic_name,time = rospy.get_time()) )
        pass

    def _callback(self, data):
        """ Description:
                callback to update values """
        """ Args:
                data(sensor_msgs.msg.Range):distance sensor data """
        self.distance = data.range
        self.max_distance = data.max_range
        self.min_distance = data.min_range
        pass

    def get_distance(self):
        """ Description:
                return distance """
        return self.distance
        pass

    def get_max_distance(self):
        """ Description:
                return max distance """
        return self.max_distance
        pass

    def get_min_distance(self):
        """ Description:
                return min distance """
        return self.min_distance
        pass

    def get_point_detected(self):
        """ Description:
                return array of point detected based on angle and position """
        point = list(self.position)
        point[0] = point[0] + self.get_distance() * math.cos(math.radians(self.angle[1])) * math.cos(math.radians(self.angle[0]))
        point[1] = point[1] + self.get_distance() * math.sin(math.radians(self.angle[1])) * math.cos(math.radians(self.angle[0]))
        point[2] = point[2] + self.get_distance() * math.sin(math.radians(self.angle[0]))
        return point
        pass


    pass
