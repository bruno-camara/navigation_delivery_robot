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
    rospy.init_node('send_goal')
    des_goal = SetGoal()
    print(des_goal.__str__())
    des_goal.go(4, 0, 0, 0, 0, 0.662, 0.750)
    
    #while not rospy.is_shutdown():
        #des_goal.go()

if __name__ == "__main__":
    try:
        main()
        pass
    except rospy.ROSInterruptException:
        pass

    finally:
        pass
