#!/usr/bin/env python
# coding=utf-8
"""
    File:
        test_distance_sensor.py
    Description:
        test for distance_sensor 
    Author:
        Vanderson Santos <vanderson.santos@usp.br>
"""
import rospy
from lib.distance_sensors import DistanceSensor

CONTROL_RATE = 60  # Hz

def main():
    rospy.init_node('test_distance_sensor', anonymous=True)
    rate = rospy.Rate(CONTROL_RATE)

    distance_sensor_back = DistanceSensor("/distance_sensor_back")
    distance_sensor_back = DistanceSensor("/distance_sensor_back")
    distance_sensor_back.initialise()
    print(distance_sensor_back)

    while not rospy.is_shutdown():
        print("distance",distance_sensor_back.get_distance())
        print("max_distance",distance_sensor_back.get_max_distance())
        print("min_distancia",distance_sensor_back.get_min_distance())
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