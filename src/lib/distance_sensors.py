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
from sensor_msgs.msg import Range

class DistanceSensor:
    def __init__(self, topic_name):
        """ Description:
                create a new Distance Sensor object"""
        """ Args:
                topic_name(string):topic name to distance sensor """
        self.distance = 0
        self.max_distance = 0
        self.min_distance = 0
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

    pass
    
