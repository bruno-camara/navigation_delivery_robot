#/usr/bin/env python
# coding=utf-8
"""
    File:
        serial_communication.py
    Description:
        serial_communication class
    Author:
        Vanderson Santos <vanderson.santos@usp.br>
    note:
        Information sender is slow, need be improve
    Data:
        08/2021
"""

import serial

class SerialCommunication:
    def __init__(self, port):
        """ Description: SerialCommunication init
            Args:
                port(string): serial port to send data
        """
        self.received_data = 0
        self.usb_port = 0
        self.port = port

    def __str__(self):
        """ Description:print Serial communication port"""
        return 'comunicação serial na porta {port}'.format(port = self.port)

    def initialise(self):
        """ Description: initialise serial communication"""
        self.usb_port = serial.Serial(port=self.port, baudrate=9600, timeout=1)

    def finalize(self):
        """ Description: finalize serial communication"""
        self.usb_port.close()

    def _callback(self, data):
        """ Description: calback to read data
            Args: data(string) data received
        """
        self.read_data()

    def read_data(self):
        """ Description: read the data received"""
        response = self.usb_port.readline()
        while(len(response)>0):
            self.received_data = response.rstrip()
            response = self.usb_port.readline()

    def get_received_data(self):
        """ Description: return last data received"""
        return self.received_data

    def send_data(self,data):
        """ Description: send data
            Args: data(string) send data do hardware"""
        self.read_data()
        if self.usb_port.isOpen():
                self.usb_port.write(data+'\r\n')