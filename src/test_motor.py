#!/usr/bin/env python
# coding=utf-8
"""
    File:
        motor.py
    Description:
        test for motor

"""
import rospy
from lib.motor import MotorControl

CONTROL_RATE = 60  # Hz

def main_read():
    rospy.init_node('test_motor', anonymous=True)
    rate = rospy.Rate(CONTROL_RATE)

    motor_back=MotorControl("/cmd_vel")
    motor_back.initialise()
    print(motor_back)

    while not rospy.is_shutdown():
        print("velocity",motor_back.get_velocity()._type)
        print("rotation",motor_back.get_rotation()._type)
        rate.sleep()
        pass

def main_set():
    rospy.init_node('test_motor', anonymous=True)
    rate = rospy.Rate(CONTROL_RATE)

    motor_back=MotorControl("/cmd_vel")
    motor_back.initialise()
    print(motor_back)

    while not rospy.is_shutdown():
        print("setting velocity 0.1",motor_back.set_velocity(0.1))
        print("setting rotation 0.1",motor_back.set_rotation(0.1))
        rate.sleep()
        pass
pass

if __name__ == "__main__":
    try:
        main_read()
        pass
    except rospy.ROSInterruptException:
        pass

    finally:
        pass
