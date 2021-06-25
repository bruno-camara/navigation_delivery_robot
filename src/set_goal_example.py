#!/usr/bin/env python

from lib.set_goal import SetGoal

des_goal = SetGoal(3, 0)
print(des_goal.__str__())
des_goal.go()