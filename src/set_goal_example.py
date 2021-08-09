#!/usr/bin/env python

"""
    File:
        test_set_goal_example.py
    Description:
        test set_goal 
    Author:
        Bruno <@usp.br>
"""

import rospy
from lib.set_goal import SetGoal

CONTROL_RATE = 60  # Hz

def main():
    rospy.init_node('send_goal', anonymous=True)
    rate = rospy.Rate(CONTROL_RATE)

    des_goal = SetGoal("/move_base") #### deixar topicos como parametro ####
    des_goal.initialise()
    print(des_goal.__str__())
    
    des_goal.go(4, 0, 0, 0, 0, 0.662, 0.750)

    while not rospy.is_shutdown():
        key = raw_input("")
        if (key == 's'):
            des_goal.stop()
        elif(key == 'q'):
            break
        

if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        des_goal.stop()
        pass

    finally:
        pass
