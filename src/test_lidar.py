#!/usr/bin/env python
# coding=utf-8
"""
    File:
        test_lidar.py
    Description:
        test for LIDAR 
    Author:
        Pedro Croso <pedrocroso@usp.br>
"""
import rospy
from lib.lidar_sensor import LidarSensor

CONTROL_RATE = 60 #Hz

def main():
    rospy.init_node('test_lidar', anonymous=True)
    rate = rospy.Rate(CONTROL_RATE)

    lidar = LidarSensor("d_hospital/laser/scan")
    lidar.initialise()
    print(lidar)

    

    while not rospy.is_shutdown():
        print "Max distance is           ", lidar.get_max_distance()
        print "Min distance is           ", lidar.get_min_distance()

        print "Closest distance is       ", lidar.get_closest_distance(-180, 180)
        print "Closest point is          ", lidar.get_closest_point(-180, 180)

        print "Closest distance left is  ", lidar.get_closest_distance(0, 180)
        print "Closest point is          ", lidar.get_closest_point(0, 180)

        print "Closest distance right is ", lidar.get_closest_distance(-180, 0)
        print "Closest point right is    ", lidar.get_closest_point(-180, 0)

        print "Normal distance left is   ", lidar.get_normal_distance_left()
        print "Normal point left is      ", lidar.get_normal_point_left()

        print "Normal distance right is  ", lidar.get_normal_distance_right()
        print "Normal point right is     ", lidar.get_normal_point_right()

        print "\n \n"

        rate.sleep()
        pass
pass

if __name__ == "__main__":
    try:
        main()
        pass
    except rospy.ROSInterruptException:
        pass
    finally:
        pass

        

       