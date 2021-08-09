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
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

CONTROL_RATE = 60  # Hz

def main():
    rospy.init_node('send_goal', anonymous=True)
    rate = rospy.Rate(CONTROL_RATE)

    des_goal = SetGoal() #### deixar topicos como parametro ####
    des_goal.initialise()
    print(des_goal.__str__())
    
    des_goal.go(0, 0, 0, 0, 0, 0.662, 0.750)

    while not rospy.is_shutdown():
        if rospy.is_shutdown():
            print("dentro if rospy.is_shutdown()")
            des_goal.stop()
            break
        

if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        print('Entrei na excecao')
        des_goal.stop()
        pass
           
    except KeyboardInterrupt:
        print("\nentrou na KeyboardInterrupt")
        des_goal.stop()
        pass

    finally:
        pass
