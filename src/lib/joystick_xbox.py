#! /usr/bin/env python
# import ros stuff

"""
    File:
        joystyck_xbox.py
    Description:
        Xbox wired controller class
    Author:
        Pedro Croso <pedrocroso@usp.br>
"""

from motor import MotorControl
import rospy
from sensor_msgs.msg import Joy


class XboxController:
    def __init__(self, joy_topic="/joy", deadzone=0.05):
        """ Description:
                create a new XboxController object"""
        """ Args:
                max_speed(float): maximum speed robot will achieve
                max_rot(float): maximum angular speed
                motor_topic(string):topic name to motor control sensor
                joy_topic(string): joystick topic
                deadzonde(float): discarted zone for analogigs in controller """
        self.deadzone = deadzone        
        self.joy_topic = joy_topic
        self.axis_left_y = 0.0
        self.axis_left_x = 0.0
        self.axis_right_y = 0.0
        self.axis_right_x = 0.0
        self.trigger_left = 0.0
        self.trigger_right = 0.0
        self.button_up_down = 0.0
        self.button_r_l = 0.0
        self.button_a = 0
        self.button_b = 0
        self.button_x = 0
        self.button_y = 0
        self.button_rb = 0
        self.button_lb = 0
        self.button_r = 0
        self.button_l = 0
        self.r_pause = 0
        self.l_pause = 0
        self.joy_axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joy_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        pass

    def initialize(self):
        """ Description:
                initialise controller subscription """
        self.joy_subscriber = rospy.Subscriber(self.joy_topic, Joy, self._callback)
        rospy.loginfo('Initialised {topic_name} at {time}'.format(topic_name = self.joy_topic,time = rospy.get_time()))
        pass

    def _callback(self, data):
        """ Description:
                callback to update values """
        """ Args:
                data(sensor_msgs.msg.Joy): joystick data """
        self.axis_left_x = data.axes[0]
        self.axis_left_y = data.axes[1]
        self.trigger_left = data.axes[2]
        self.axis_right_x = data.axes[3]
        self.axis_right_y = data.axes[4]
        self.trigger_right = data.axes[5]
        self.button_r_l = data.axes[6]
        self.button_up_down = data.axes[7]
        self.button_a = data.buttons[0]
        self.button_b = data.buttons[1]
        self.button_x = data.buttons[2]
        self.button_y = data.buttons[3]
        self.button_lb = data.buttons[4]
        self.button_rb = data.buttons[5]
        self.l_pause = data.buttons[6]
        self.r_pause = data.buttons[7]
        self.button_l = data.buttons[9]
        self.button_r = data.buttons[10]
        
        self.joy_axes = data.axes
        self.joy_buttons = data.buttons
        pass


    def get_left_annalogic(self):
        """ Description:
                return a list with x axis(0) and y axis(1) from left analogic from 0 to 1 """
        analogic = [self.axis_left_x, self.axis_left_y]
        return analogic
        pass

    def get_right_analogic(self):
        """ Description:
                return a list with x axis(0) and y axis(1) from right analogic from 0 to 1 """
        analogic = [self.axis_right_x, self.axis_right_y]
        return analogic
        pass

    def get_left_trigger(self):
        """ Description:
                return left trigger value from -1 to 1 """
        return self.trigger_left
        pass

    def get_right_trigger(self):
        """ Description:
                return right trigger value from -1 to 1 """
        return self.trigger_right
        pass

    def get_r_l_up_down_button(self):
        """ Description:
                return a list with the right, left, up and down press """
        buttons = [0,0,0,0]
        if self.button_r_l < -0.1:
            buttons[0] = 1
        elif self.button_r_l > 0.1:
            buttons[1] = 1
        
        if self.button_up_down > 0.1:
            buttons[2] = 1
        elif self.button_up_down < -0.1:
            buttons[3] = 1
        
        return buttons
        pass

    def get_abxy_buttons(self):
        """ Description:
                return a list with the A, B, X and Y press """
        buttons = [self.button_a, self.button_b, self.button_x, self.button_y]
        return buttons
        pass

    def get_rb_lb(self):
        """ Description:
                return a list with the lb and rb press """
        buttons = [self.button_lb, self.button_rb]
        return buttons
        pass

    def get_r_l_pause(self):
        """ Description:
                return a list with the left and right pause press """
        buttons = [self.l_pause, self.r_pause]
        return buttons
        pass

    def get_analogic_press(self):
        """ Description:
                return a list with the left and right analogics press """
        buttons = [self.button_l, self.button_r]
        return buttons
        pass


    

