#/usr/bin/env python
# coding=utf-8
"""
    File:
        serial_communication.py
    Description:
        serial_communication class
    Author:
        Vanderson Santos <vanderson.santos@usp.br>
    Data:
        08/2021
"""

import serial

class SerialCommunication:
    def __init__(self, port):
        self.received_data = 0
        self.usb_port = 0
        self.port = port

    def __str__(self):
        return 'comunicação serial na porta {port}'.format(port = self.port)

    def initialise(self):
        self.usb_port = serial.Serial(port=self.port, baudrate=9600, timeout=1)

    def finalize(self):
        self.usb_port.close()

    def _callback(self, data):
        self.read_data()

    def read_data(self):
        #read the output and store it in a variable
        response = self.usb_port.readline()
        #Check to see if all the lines of the output are printed
        while(len(response)>0):
            self.received_data = response.rstrip()
            response = self.usb_port.readline()

    def get_received_data(self):
        return self.received_data

    def send_data(self,data):
        self.read_data()
        #check to see if the connection is open
        if self.usb_port.isOpen():
                #while true, send 'r' char to receive the output from the sensor
                self.usb_port.write(data+'\r\n')
                #loop to print all the output lines