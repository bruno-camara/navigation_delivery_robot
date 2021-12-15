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

from numpy.core.fromnumeric import size
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LidarSensor:
    def __init__(self, topic_name, n_regions = 720, angle_offset = 0.0):
        """ Description:
                create a new LIDAR Sensor object"""
        """ Args:
                topic_name(string):topic name to lidar sensor
                n_regions: number of rays per revolutions
                angle offset: angle of lidar in reference to its system """

        self.closest_distance = 0.0
        self.closest_point = np.array([0.0, 0.0])

        self.normal_distance_left = 0.0
        self.normal_point_left = np.array([0.0, 0.0])
        self.normal_distance_right = 0.0
        self.normal_point_right = np.array([0.0, 0.0])

        self.topic_name = topic_name

        self.max_distance = 0.0
        self.min_distance = 0.0

        self.regions = [0.0] * n_regions
        self.number_of_reads = n_regions
        self.d_th = 360.0/self.number_of_reads

        self.angle_offset = angle_offset

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
	#print ("Size of lidar msg")
	#print(size(data.ranges))
        #regions_temp = [0.0] * self.number_of_reads
        for i in range(0, min(self.number_of_reads, size(data.ranges)), 1):
            self.regions[i] = max(min(data.ranges[i], self.max_distance), self.min_distance)

<<<<<<< HEAD
        index_offset = int(self.angle_offset*self.number_of_reads/360)
	print("Offset: ")
	print(index_offset)
        if self.angle_offset < 0:
=======
        index_offset = int(360*self.angle_offset/self.number_of_reads)
        if self.angle_offset < 0.0:
>>>>>>> 17b4d6a48eed34888fbe921496f871ba8842ed0c
            self.regions = self.regions[index_offset:] + self.regions[:index_offset]
        elif self.angle_offset > 0.0:
            self.regions = self.regions[:index_offset] + self.regions[index_offset:]


        #if self.angle_offset > 0:
        #    index_offset = (self.angle_offset/self.number_of_reads)
        #    for i in range(index_offset, self.number_of_reads):
        #        self.regions[i]=regions_temp[i-index_offset]
        #    for i in range(0,index_offset):
        #        self.regions[i] = regions_temp[(self.number_of_reads-index_offset)+i]
        #elif self.angle_offset < 0:
        #    index_offset = - (self.angle_offset/self.number_of_reads)
        #    for i in range(index_offset, self.number_of_reads):
        #        self.regions[i-index_offset] = regions_temp[i]
        #    for i in range(0, index_offset):
        #        self.regions[i+index_offset] = regions_temp[i]


        self.normal_distance_left = min(self.regions[int(0.5*self.number_of_reads)-1:int(0.5 * self.number_of_reads)+1])
        self.normal_point_left = np.array([-self.normal_distance_left, 0.0])

        self.normal_distance_right = min(self.regions[self.number_of_reads-1], self.regions[1], self.regions[0])
        self.normal_point_right = np.array([self.normal_distance_right, 0.0])


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
        start_data = int(min_angle / self.d_th + (self.number_of_reads/2))
        stop_data = int(max_angle / self.d_th + (self.number_of_reads/2))
        #print(min_angle, max_angle)

        #print(start_data)
        #print(stop_data)
        #print(self.regions)


        self.closest_distance = min(min(self.regions[start_data : stop_data]), self.max_distance)
        return self.closest_distance
        pass

    def get_closest_point(self, min_angle, max_angle):
        """ Description:
                gives the array of closest point in a given angle range """
        """ Args:
                self, start angle, stop angle: given in degrees, 0 is in front of LIDAR,
                counter clockwise is positive """
        start_data = int(min_angle / self.d_th + (self.number_of_reads/2)) #int(min_angle / self.d_th + (self.number_of_reads/2))
        stop_data = int(max_angle / self.d_th + (self.number_of_reads/2)) #int(max_angle / self.d_th + (self.number_of_reads/2))
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

    def get_closest_point_angle(self, min_angle, max_angle):
        """ Description:
                gives the angle of closest point in a given angle range """
        """ Args:
                self, start angle, stop angle: given in degrees, 0 is in front of LIDAR,
                counter clockwise is positive """
        start_data = int(min_angle / self.d_th + (self.number_of_reads/2))#int(min_angle / self.d_th + (self.number_of_reads/2))
        stop_data = int(max_angle / self.d_th + (self.number_of_reads/2))#int(max_angle / self.d_th + (self.number_of_reads/2))
        index = self.regions.index(min(self.regions[start_data : stop_data]), start_data, stop_data)
        angle = (index - 360) / 2
        return angle
        pass
