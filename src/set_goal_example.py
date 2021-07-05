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
    des_goal = SetGoal(4, 0)
    print(des_goal.__str__())
    
    while not rospy.is_shutdown():
        des_goal.go()

if __name__ == "__main__":
    try:
        main()
        pass
    except rospy.ROSInterruptException:
        pass

    finally:
        pass
