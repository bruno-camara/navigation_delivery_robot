import rospy
from lib.motor import MotorControl
from lib.serial_communication import SerialCommunication

import numpy as np

CONTROL_RATE = 60  # Hz

MIN = -1
MAX = 1

NEW_MIN = 0
NEW_MAX = 1023

def map(value,min,max,new_min,new_max):
    new_value = (value-min)*(new_max-new_min)
    new_value /= (max-min)
    new_value += min
    return new_value

def robot_manual_controller():

    info_serial_vel = map(motor_back.get_velocity(),MIN,MAX,NEW_MIN,NEW_MAX)
    info_serial_rot = map(motor_back.get_rotation(),MIN,MAX,NEW_MIN,NEW_MAX)

    

if __name__ == "__main__":
    try:
        robot_manual_controller()
        pass
    except rospy.ROSInterruptException:
        pass

    finally:
        pass