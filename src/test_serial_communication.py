#/usr/bin/env python
# coding=utf-8
"""
    File:
        test_serial_communication.py
    Description:
        test to serial_communication class
    Author:
        Vanderson Santos <vanderson.santos@usp.br>
    Data:
        08/2021
"""

import rospy
from lib.serial_communication import SerialCommunication

PORT = "/dev/ttyUSB0"

def main():
    communication = SerialCommunication(PORT)
    communication.initialise()

    communication.send_data('10001')
    communication.send_data('11001')
    communication.send_data('10001')
    communication.send_data('11001')

    communication.finalize()

if __name__ == "__main__":
    try:
        main()
        pass
    finally:
        pass