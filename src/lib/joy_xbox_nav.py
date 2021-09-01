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
    def __init__(self, max_speed=0.5, max_rot=0.5, motor_topic="/cmd_vel", joy_topic="/joy", deadzone=0.05):
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

    def initialize(self):
        """ Description:
                initialise odometry and controller """
        rospy.init_node("joystick_navigation")
        self.control.initialise()
        self.joystick.initialize()
        pass

    def set_max_vel(self):
        """ Description:
                asks and update max_vel """
        is_valid = False
        while not is_valid:
            try:
                new_max_speed = float(input("Max linear speed: "))
                self.max_speed = new_max_speed
                is_valid = True
            except:
                print "Please, enter a valid value"
        pass

    def set_max_rot(self):
        """ Description:
                asks and update max_rot """
        is_valid = False
        while not is_valid:
            try:
                new_max_rot = float(input("Max angular speed: "))
                self.max_rot = new_max_rot
                is_valid = True
            except:
                print "Please enter a valid value"
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
    js.initialize()
    js.print_info()
    js.set_max_vel()
    js.set_max_rot()
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        js.set_vel()
        js.set_rot()
        rate.sleep

if __name__ == '__main__':
    main()
