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
#init serial port and bound
# bound rate on two ports must be the same
#ser = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
#print(ser.portstr)
#
##send data via serial port
#ser.write('0')
#ser.write('1')
#ser.write('0')
#ser.write('0')
#ser.write('0')
#ser.write('\n')
#ser.close()
    
#ser.write(b'5') #Prefixo b necessario se estiver utilizando Python 3.X 
#with serial.Serial('/dev/ttyUSB0', 19200, timeout=1) as ser:
#    info = "00020"
#    ser = serial.Serial('/dev/ttyUSB0')  # open serial port
#    print(ser.name)         # check which port was really used
#    ser.write(info)         # write a string
#      ser.close()  

import serial

#Open a serial connection between Omega and ATmega
new_port = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

def read_data():
    #read the output and store it in a variable
    response = new_port.readline()
    #Check to see if all the lines of the output are printed
    while(len(response)>0):
        print (response.rstrip())
        response = new_port.readline()
        

def get_temperature(new_port):

    read_data()
    #check to see if the connection is open
    if new_port.isOpen():
            #while true, send 'r' char to receive the output from the sensor
            new_port.write('10001\n')
            #loop to print all the output lines

#Function call
get_temperature(new_port)

#close the serial connection after the function is over
new_port.close()