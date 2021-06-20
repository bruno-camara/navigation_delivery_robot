#/usr/bin/env python
# coding=utf-8
"""
    File:
        lidar_sensor.py
    Description:
        lidar_sensor object
    Author:
        Pedro Croso <pedrocroso@usp.br>
"""

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LidarSensor:
    def __init__(self, topic_name):
        """ Description:
                create a new LIDAR Sensor object"""
        """ Args:
                topic_name(string):topic name to lidar sensor """
        self.regions = [0.0] * 720

        self.closest_distance = 0.0
        self.closest_point = np.array([0.0, 0.0])

        self.normal_distance_left = 0.0
        self.normal_point_left = np.array([0.0, 0.0])
        self.normal_distance_right = 0.0
        self.normal_point_right = np.array([0.0, 0.0])

        self.topic_name = topic_name

        self.max_distance = 0.0
        self.min_distance = 0.0
        pass
    
    def __str__(self):
        """ Description:
                print lidar sensor topic name"""
        return 'sensor topic name = {topic_name}'.format(topic_name = self.topic_name)
        pass
    
    def initialise(self):
        """ Description:
                initialise lidar sensor """
        self.subscriber = rospy.Subscriber(self.topic_name, LaserScan, self._callback)
        rospy.loginfo('Initialised {topic_name} at {time}'.format(topic_name = self.topic_name,time = rospy.get_time()))
        pass
    
    def _callback(self, data):
        """ Description:
                callback to update values """
        """ Args:
                data(sensor_msgs.msg.LaserScan):lidar sensor data """
        self.max_distance = data.range_max
        self.min_distance = data.range_min
        for i in range(0, 720, 1):
            self.regions[i] = max(min(data.ranges[i], self.max_distance), self.min_distance)
        
        self.normal_distance_left = min(self.regions[538:542])
        self.normal_point_left = np.array([-min(self.regions[538:542]), 0.0])

        self.normal_distance_right = min(self.regions[178:182])
        self.normal_point_right = np.array([min(self.regions[178:182]), 0.0])
        
        
    def get_regions(self):
        return self.regions
        pass
    
    def get_max_distance(self):
        return self.max_distance
        pass
    
    def get_min_distance(self):
        return self.min_distance
        pass

    
    def get_closest_distance(self, min_angle, max_angle):
        """ Description:
                gives the minimum distance in a given angle range """
        """ Args:
                self, start angle, stop angle: given in degrees, 0 is in front of LIDAR,
                counter clockwise is positive """
        start_data = 360 + 2 * min_angle
        stop_data = 360 + 2 * max_angle
        self.closest_distance = min(min(self.regions[start_data : stop_data]), self.max_distance)
        return self.closest_distance
        pass
    
    def get_closest_point(self, min_angle, max_angle):
        """ Description:
                gives the array of closest point in a given angle range """
        """ Args:
                self, start angle, stop angle: given in degrees, 0 is in front of LIDAR,
                counter clockwise is positive """
        start_data = 360 + 2 * min_angle
        stop_data = 360 + 2 * max_angle
        distance = self.get_closest_distance(min_angle, max_angle)
        self.closest_point = distance * np.array([np.cos(math.radians((self.regions.index(distance, start_data, stop_data)-180)/2)), 
                             np.sin(math.radians((self.regions.index(distance, start_data, stop_data)-180)/2))])
        
        return self.closest_point
        pass
    
    def get_normal_distance_left(self):
        """ Description:
                gives the distance taken in 90 degrees """
        return self.normal_distance_left 
        pass
    
    def get_normal_distance_right(self):
        """ Description:
                gives the distance taken in -90 degrees """
        return self.normal_distance_right
        pass
    
    def get_normal_point_left(self):
        """ Description:
                gives the array taken in 90 degrees """
        return self.normal_point_left
        pass

    def get_normal_point_right(self):
        """ Description:
                gives the array taken in -90 degrees """
        return self.normal_point_right
        pass

        
