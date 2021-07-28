#/usr/bin/env python
# coding=utf-8
"""
    File:
        distance_sensor.py
    Description:
        distance_sensor object
    Author:
        Vanderson Santos <vanderson.santos@usp.br>
    Data:
        07/2021
"""

import serial
while (True):
    info = "00001"
    ser = serial.Serial('/dev/ttyUSB0', 9600,timeout=1)
    ser.write(info)
    msg = ser.read()
    print(msg)
    
#ser.write(b'5') #Prefixo b necessario se estiver utilizando Python 3.X 
#with serial.Serial('/dev/ttyUSB0', 19200, timeout=1) as ser:
#    info = "00020"
#    ser = serial.Serial('/dev/ttyUSB0')  # open serial port
#    print(ser.name)         # check which port was really used
#    ser.write(info)         # write a string
#      ser.close()  