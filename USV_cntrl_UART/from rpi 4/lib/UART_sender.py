#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UART Module: Sends u_actual and motor_speed from SimData over UART.

Author: Your Name
"""

import serial
import time
import numpy as np
import math
import json

class UART_sender:
    def __init__(self, port='/dev/ttyS0', baudrate=9600, timeout=1):
        """
        Initialize the UART connection.

        :param port: Serial port for UART (default: '/dev/ttyS0' for Raspberry Pi)
        :param baudrate: Baud rate for UART communication
        :param timeout: Timeout for UART communication
        """
        try:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
            print(f"UART initialized on {port} with baudrate {baudrate}")
        except serial.SerialException as e:
            print(f"Error initializing UART: {e}")
            self.serial = None

    def send_data(self, u_actual, motor_speed):
        """
        Send u_actual and motor_speed via UART.

        :param u_actual: Control input (e.g., rudder angle)
        :param motor_speed: Speed of the motors
        """
        if not self.serial or not self.serial.is_open:
            print("UART connection is not open.")
            return

        # Prepare the data in a comma-separated format
        data_dict = {
            "u_actual": u_actual,
            "motor_speed": motor_speed
        }
        data = json.dumps(data_dict) + "\n"  # Преобразование в JSON и добавление символа новой строки
        try:
            self.serial.write(data.encode('utf-8'))
            print(f"Sent data: {data.strip()}")
        except serial.SerialException as e:
            print(f"Error sending data: {e}")

    def close(self):
        """
        Close the UART connection.
        """
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("UART connection closed.")

# Function to process SimData and send relevant data
def process_and_send(simData, uart_sender):
    """
    Process SimData and send u_actual and motor_speed over UART.

    :param simData: Simulation data (numpy array)
    :param uart_sender: Instance of UART_sender
    """
    nmax_motor_speed = 1500
    deltaMax = 35
    motor_efficiency = 0.85
    V_c = 3.5
    
    for row in simData:
        u_actual = row[13]*180/math.pi
        motor_speed = nmax_motor_speed * (V_c / 6.0) * motor_efficiency # (1 - abs(u_actual) / deltaMax) 
        uart_sender.send_data(u_actual, motor_speed)
        time.sleep(0.02)  # Match the simulation sample time
    uart_sender.send_data(0,0)
        
def process_and_send_turn(simData, uart_sender):
    """
    Process SimData and send u_actual and motor_speed over UART.

    :param simData: Simulation data (numpy array)
    :param uart_sender: Instance of UART_sender
    """
    nmax_motor_speed = 1500
    deltaMax = 35
    motor_efficiency = 0.85
    V_c = 3.5
    
    for el in simData:
        u_actual = el#*180/math.pi
        motor_speed = nmax_motor_speed * (V_c / 6.0) * motor_efficiency # (1 - abs(u_actual) / deltaMax) 
        uart_sender.send_data(u_actual, motor_speed)
        time.sleep(0.02)  # Match the simulation sample time
    uart_sender.send_data(0,0)

