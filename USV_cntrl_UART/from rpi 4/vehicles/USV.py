#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
frigate.py:  

    Class for the frigate (Length 100 m) based on the  Norrbin (1963) 
    nonlinear autopilot model

                      psi_dot = r                     - yaw kinematics
      T r_dot + n3 r^3 + n1 r = K delta + d_r         - Norrbin (1963) model
                    dot_delta = f(delta, delta_c)     - rudder dynamics
     
    The ship is controlled by a controllable pitch propeller and a rudder with 
    rudder dynamics. The model parameters K, T and n3 are speed dependent, 
    while n = 1 (course stable ship). The parameter sare interpolated for 
    varying speeds U. The velocity state vector is nu  = [ 0 0 0 0 0 r]' 
    where r is the yaw vwlocity (rad/s). The constructors are:
        
    frigate()                      
        Step input, rudder angel
        
    frigate('headingAutopilot',psi_d)  
         Heading autopilot with option:
            psi_d: desired heading (m)
        
Methods:   
        
    [nu, u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime) returns
         nu[k+1] and u_actual[k+1] using Euler's method. The control input is:
       
        u_control = delta (rad):  rudder angle

    u = headingAutopilot(eta,nu,sampleTime) 
        PID controller for automatic heading control based on pole placement.
       
        u = stepInput(t) generates rudder angle step inputs.   
       
References: 
    
    J. Van Amerongen (1982). Adaptive Steering of Ships – A Model Reference 
          Approach to Improved Maneuvering and Economical Course Keeping. PhD 
          thesis. Delft University of Technology, Netherlands.
    T. I. Fossen (2021). Handbook of Marine Craft Hydrodynamics and Motion 
         Control. 2nd. Edition, Wiley. URL: www.fossen.biz/wiley            

Author:     Thor I. Fossen
"""
import numpy as np
import math
import sys
from lib.control import PIDpolePlacement

# Class Vehicle
class USV:
    """
    frigate()                               Step input, rudder angle
    frigate('headingAutopilot',U,psi_d)     Heading autopilot
    
    Inputs:
        U: cruise speed (m/s)
        psi_d: desired heading angle (deg)                                           and desired heading(deg)
    """        
    def __init__(self, controlSystem = 'stepInput', U = 5.0, r = 0.0):
                                  
        if (controlSystem == 'headingAutopilot'):
            self.controlDescription = (
                'Heading autopilot, psi_d = ' 
                + str(r) 
                + ' deg'
                )
             
        else:  
            self.controlDescription = "Step input for delta" 
            controlSystem = 'stepInput'  
      
        # Check if the speed U is within the valid range
        if (U < 1.0 or U > 6.0):
            sys.exit('The speed U should be between 1-6 m/s')          
            
        self.ref = r
        self.controlMode = controlSystem
                    
        # Initialize the ship model
        self.name = ""
        self.L = 1.225 #100.0        # Length      
        self.deltaMax = 35#30   # max rudder angle (deg)  
        self.DdeltaMax = 20#10  # max rudder rate (deg/s)        
        self.nu  = np.array([ U, 0, 0, 0, 0, 0],float)  # velocity vector  
        self.u_actual = np.array([0],float)             # control input vector              

        self.controls = ['Rudder angle (deg)']
        self.dimU = len(self.controls)
        
        #motors
        self.max_motor_speed = 1500  # максимальная скорость двигателя (об/мин)
        self.motor_efficiency = 0.85  # эффективность преобразования энергии в скорость

        self.F = True

        # ROV Zefakkel (Van Amerongen 1982) 
        self.n1 = 0.5
        self.n3 = 0.3
        
        # interpolate to find K, T and n3 as a function of U
        U_data = np.array([1.0, 3.5, 6.0], float)#([ 6.0, 9.0, 12.0 ],float)
        K_data = np.array([ 0.15, 0.2, 0.16],float)#[ 2, 2.2, 3],float)
        T_data = np.array([0.1, 0.8, 0.18],float) #[ 20.0, 27.0, 21.0 ],float) 
        n3_data = np.array([ 0.4, 0.3, 0.4 ],float)         
        

        U = self.nu[0]
        self.K  = np.interp( U, U_data, K_data )
        self.T  = np.interp( U, U_data, T_data )
        self.n3 = np.interp( U, U_data, n3_data )
    
        # Heading autopilot  
        self.e_int = 0.0#0.0     # integral state, initial value
        self.wn = 1.3#1.9            # PID pole placement parameters
        self.zeta = 1#1.3
        
        # Reference model
        self.r_max = 10 * math.pi / 180   # maximum yaw rate 
        self.psi_d = 0                   # position, velocity and acc. states
        self.r_d = 0
        self.a_d = 0
        self.wn_d = self.wn / 5
        self.zeta_d = 1   
        

        
    def dynamics(self,eta,nu,u_actual,u_control,sampleTime):
        """
        [nu, u_actual] = dynamics(eta,nu,u_actual,u_control,sampleTime)
        integrates the ship equations of motion using Euler's method.
        """       

        # States and inputs
        delta_c = u_control[0]
        delta   = u_actual[0]   
        r       = nu[5]
        
        # Rudder angle saturation and dynamics
        if ( abs(delta) >= self.deltaMax * math.pi/180 ):
            delta = np.sign(delta) * self.deltaMax * math.pi/180

        delta_dot = delta_c - delta
        if ( abs(delta_dot) >= self.DdeltaMax * math.pi/180 ):
            delta_dot = np.sign(delta_dot) * self.DdeltaMax * math.pi/180
                    
        # Dynamics
        r_dot = (1 / self.T) * ( self.K * delta - self.n3 * r**3 - self.n1 * r )
        nu_dot = np.array( [0, 0, 0, 0, 0, r_dot], float)
        
        # Forward Euler integration [k+1]
        nu  = nu + sampleTime * nu_dot
        delta = delta + sampleTime * delta_dot
        
        motors_speed = self.calculate_motors_speed(delta, nu[0])
        if self.F == True: 
            print("Скороть двигателей: ", np.round(motors_speed,0),"об/мин")
            self.F =  False
        u_actual = np.array([delta],float)          
        
        return nu, u_actual
    
    
    def stepInput(self,delta_c, t):
        """
        delta_c = stepInput(t) generates stern plane step inputs.
        """    
        #delta_c = 35 * (math.pi/180)    
        if t > 50:
            delta_c = 0
        #if t > 100:
        #    delta_c = 0
            
        u_control = np.array([delta_c],float)   
         
        return u_control             


    def headingAutopilot(self,eta,nu,sampleTime):
        """
        u = headingAutopilot(eta,nu,sampleTime) is a PID controller 
        for automatic heading control based on pole placement.
        
        delta = (T/K) * a_d + (1/K) * rd 
               - Kp * ( ssa( psi-psi_d ) + Td * (r - r_d) + (1/Ti) * z )
        
        """                  
        psi = eta[5]                # yaw angle
        r = nu[5]                   # yaw rate
        e_psi = psi - self.psi_d    # yaw angle tracking error
        e_r   = r - self.r_d        # yaw rate tracking error
        psi_ref = self.ref * math.pi / 180  # yaw angle setpoint
    
        wn = self.wn                # PID natural frequency
        zeta = self.zeta            # PID natural relative damping factor
        wn_d = self.wn_d            # reference model natural frequency
        zeta_d = self.zeta_d        # reference model relative damping factor

        m = self.T / self.K#0.7            
        d = self.n1 / self.K   #0.4
        k = 0.1 #5

        # PID feedback controller with 3rd-order reference model
        [delta, self.e_int, self.psi_d, self.r_d, self.a_d] = \
            PIDpolePlacement( self.e_int, e_psi, e_r, self.psi_d, self.r_d, self.a_d, \
            m, d, k, wn_d, zeta_d, wn, zeta, psi_ref, self.r_max, sampleTime )

        u_control = np.array([delta],float)   
         
        return u_control     

    def calculate_motors_speed(self, delta, U):
        """
        Calculate engine speed based on rudder angle (delta) and cruise speed (U).
        """
        # скорость двигателя зависит от угла руля и текущей скорости
        motors_speed = self.max_motor_speed * (1 - abs(delta) / self.deltaMax) * (U / 6.0) * self.motor_efficiency
        return motors_speed