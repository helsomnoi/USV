#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py: Main program for the Python Vehicle Simulator, which can be used
    to simulate and test guidance, navigation and control (GNC) systems.

Reference: T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and
Motion Control. 2nd. Edition, Wiley. 
URL: www.fossen.biz/wiley  
    
Author:     Thor I. Fossen
"""
import os
import webbrowser
import matplotlib.pyplot as plt
from vehicles import *
from lib import *

# Simulation parameters: 
sampleTime = 0.02                   # sample time
N = 2000                          # number of samples

# 3D plot and animation parameters where browser = {firefox,chrome,safari,etc.}
numDataPoints = 50                  # number of 3D data points
FPS = 10                            # frames per second (animated GIF)
filename = '3D_animation.gif'       # data file for animated GIF
browser = 'safari'                  # browser for visualization of animated GIF

###############################################################################
# Vehicle constructors
###############################################################################
printSimInfo() 

"""
                                
frigate('headingAutopilot',U,psi_d)

"""
psi_d = 90

#vehicle = frigate('stepInput',60) 
vehicle = USV('headingAutopilot',3.5,psi_d)
printVehicleinfo(vehicle, sampleTime, N)

###############################################################################
# Main simulation loop 
###############################################################################
def main():    
    
    [simTime, simData] = simulate(N, sampleTime, vehicle)
    
    plotVehicleStates(simTime, simData, 1, psi_d)                    
    #plotControls(simTime, simData, vehicle, 2)
    plot3D(simData, numDataPoints, FPS, filename, 2)   
    
    """ Ucomment the line below for 3D animation in the web browswer. 
    Alternatively, open the animated GIF file manually in your preferred browser. """
    #webbrowser.get(browser).open_new_tab('file://' + os.path.abspath(filename))
    
    plt.show()
    plt.close()

    uart_sender = UART_sender(port='/dev/ttyS0', baudrate=115200)

    try:
        # Process and send data from simData
        process_and_send(simData, uart_sender)
    finally:
        # Close UART connection
        uart_sender.close()

main()
