#!/usr/bin/env python
# coding=utf-8
"""
    File:
        motor.py
    Description:
        test for motor 
 
"""
import rospy
from lib.motor import MotorControler

CONTROL_RATE = 60  # Hz

def main_read():
    rospy.init_node('test_motor', anonymous=True)
    rate = rospy.Rate(CONTROL_RATE)

    motor_back=MotorControler("/motor_back")
    motor_back.initialise()
    print(motor_back)

    while not rospy.is_shutdown():
        print("velocity",motor_back.get_velocity())
        print("rotation",motor_back.get_rotation())
        rate.sleep()
        pass

def main_set():
    rospy.init_node('test_motor', anonymous=True)
    rate = rospy.Rate(CONTROL_RATE)

    motor_back=MotorControler("/motor_back")
    motor_back.initialise()
    print(motor_back)

    while not rospy.is_shutdown():
        print("setting velocity 15",motor_back.set_velocity(15))
        print("setting rotation 15",motor_back.set_rotation(15))
        rate.sleep()
        pass
pass

if __name__ == "__main__":
    try:
        main_set()
        pass
    except rospy.ROSInterruptException:
        pass

    finally:
        pass