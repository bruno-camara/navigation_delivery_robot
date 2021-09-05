#! /usr/bin/env python
# import ros stuff

"""
    File:
       test_kb_control.py
    Description:
        tests Keyboard controll class for D_hospital robot
    Author:
        Pedro Croso <pedrocroso@usp.br>
"""

from lib.keyboard_nav import KeyboardControler


def main():
    kb = KeyboardControler()
    kb.initialize()
    kb.print_info()
    kb.set_max_vel()
    kb.set_max_rot()
    kb.go_continuous()
    print "See ya!"
    return
    pass





if __name__ == '__main__':
    main()
