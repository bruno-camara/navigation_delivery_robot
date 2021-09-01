#! /usr/bin/env python
# import ros stuff

"""
    File:
       keyboard_nav.py
    Description:
        Keyboard controll class for D_hospital robot
    Author:
        Pedro Croso <pedrocroso@usp.br>
"""

from motor import MotorControl
import rospy
from pynput import keyboard

class KeyboardControler:
    def __init__(self, max_speed=0.5, max_rot=0.5, topic="/cmd_vel"):
        """ Description:
                creates a new KeyboardController object"""
        """ Args:
                max_speed(float): maximum speed robot will achieve
                max_rot(float): maximum angular speed
                motor_topic(string):topic name to motor control sensor """
        self.max_speed = max_speed
        self.max_rot = max_rot
        self.topic = topic
        self.control = MotorControl(topic)
        pass

    def initialize(self):
        """ Description:
                initialise odometry and keyboard control """
        rospy.init_node("kb_joystick")
        self.control.initialise()
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



    def set_vel(self, command):
        """ Description:
                updates robot speed acording to keyboard data """
        if command == 'w':
            self.control.set_velocity(self.max_speed)
        elif command == 's':
            self.control.set_velocity(-self.max_speed)
        pass

    def set_rot(self, command):
        """ Description:
                updates robot angular speed acording to keyboard data """
        if command == 'a':
            self.control.set_rotation(self.max_rot)
        elif command == 'd':
            self.control.set_rotation(-self.max_rot)
        pass

    def go_continuous(self):
        """ Description:
                initialisaes contininuous behavior on reading from keyboard
                exits function when key 'p' is released """
        def pressing(key):
            try:
                command = key.char
            except:
                return True

            if command not in ['a', 's', 'd', 'w']:
                return True
            else:
                self.set_rot(command)
                self.set_vel(command)

        def releasing(key):
            try:
                released = key.char
                print released
            except:
                return True
            print "Rotation: "
            print self.control.get_rotation()
            print "Velocity: "
            print self.control.get_velocity()
            if (self.control.get_velocity().x > 0.0) and (released == 'w'):
                self.control.set_velocity(0.0)
            elif (self.control.get_velocity().x < 0.0) and (released == 's'):
                self.control.set_velocity(0.0)
            if (self.control.get_rotation().z > 0.0) and (released == 'a'):
                self.control.set_rotation(0.0)
            elif(self.control.get_rotation().z < 0.0) and (released == 'd'):
                self.control.set_rotation(0.0)
            if released == 'p':
                return False

        with keyboard.Listener(on_press=pressing, on_release=releasing) as lis:
            lis.join()
        pass

    def print_info(self):
        """ Description:
                print some usefull info in screen """
        print "Welcome to the DelHospital controller!"
        print "To navigate you can use the keys"
        print "   w   "
        print "a  s  d \n\n"
        print "To kill the program, press and release the key 'P' on your keyboard \n\n"
        print "But first of all, please set the max speed and max rotation for the robot"
