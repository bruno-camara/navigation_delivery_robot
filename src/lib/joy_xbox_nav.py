#! /usr/bin/env python
# import ros stuff

"""
    File:
        joystyck_xbox.py
    Description:
        Control the robot using a Xbox wired controller
    Author:
        Pedro Croso <pedrocroso@usp.br>
"""


from motor import MotorControl
import rospy
from joystick_xbox import XboxController

class JoyNav:
    def __init__(self, max_speed=0.5, max_rot=0.5, motor_topic="/cmd_vel", joy_topic="/joy", deadzone=0.1):
        """ Description:
                creates a new JoyNav object"""
        """ Args:
                max_speed(float): maximum speed robot will achieve
                max_rot(float): maximum angular speed
                motor_topic(string):topic name to motor control sensor
                joy_topic(string): joystick topic
                deadzonde(float): discarted zone for analogigs in controller """
        self.max_speed = max_speed
        self.max_rot = max_rot
        self.control = MotorControl(motor_topic)
        self.joystick = XboxController(joy_topic, deadzone)
        pass

    def initialise(self):
        """ Description:
                initialise odometry and controller """
        rospy.init_node("joystick_navigation")
        self.control.initialise()
        self.joystick.initialise()
        pass

    def set_max_vel(self, new_speed = 1):
        """ Description:
                updates max_vel """
        """ Args:
                new_speed(float): new maximum speed. Deafault is 1"""
        is_valid = False
        while not is_valid:
            try:
                new_max_speed = float(new_speed)
                self.max_speed = new_max_speed
                is_valid = True
            except ValueError:
                print "Error setting max speed"
        pass

    def get_max_vel(self):
        """ Description:
                returns max_vel """
        return self.max_speed
        pass


    def set_max_rot(self, new_rot = 1):
        """ Description:
                asks and update max_rot """
        """ Args:
                new_rot(float): new maximum rotation. Deafault is 1"""
        is_valid = False
        while not is_valid:
            try:
                new_max_rot = float(new_rot)
                self.max_rot = new_max_rot
                is_valid = True
            except ValueError:
                print "Error setting max rotation"
        pass

    def get_max_rot(self):
        """ Description:
                returns max_rot """
        return self.max_rot
        pass

    def set_vel(self):
        """ Description:
                updates robot speed acording to joystick data """
        self.control.set_velocity(self.max_speed * self.joystick.get_left_annalogic()[1])
        pass

    def set_rot(self):
        """ Description:
                updates robot rotation acording to joystick data """
        self.control.set_rotation(self.max_rot * self.joystick.get_right_analogic()[0])
        pass

    def print_info(self):
        print "Welcome to the DelHospital controller!"
        print "To navigate you can use the Xbox controller"
        print "But first of all, please set the max speed and max rotation for the robot"
        pass


def main():
    js = JoyNav()
    js.initialise()
    js.print_info()
    js.set_max_vel(input("Enter maximum speed "))
    js.set_max_rot(input("Enter maximum rotation "))
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        js.set_vel()
        js.set_rot()
        rate.sleep

if __name__ == '__main__':
    main()
