
SQUID DRONE CURRICULUM: lab_3_1_sysid.py
----------------------------------
Context: ../../../../curriculum/Phases/Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Lecture.md
Theory: curriculum/Library/
Win Condition: Phase III: The Notch Filter (SQUID_GAMES.md)

This lab is part of the 'Soldering Iron to Swarm' masterclass.

"""
Lab 4.3: System Identification (SysID)
Goal: Find the Thrust Constant (Kt) of your Pavo20.
Refer to docs/theory/Theory_0.4_System_Identification.md
"""

import numpy as np
import matplotlib.pyplot as plt

def calculate_thrust_constant(pwm_values, measured_mass_grams):
    """
    Inputs:
    pwm_values: List of throttle signals [0.0 to 1.0]
    measured_mass_grams: List of scale readings in grams
    
    Output:
    Kt: The constant where Thrust = Kt * (PWM^2) or Kt * PWM
    """
    # 1. Convert grams to Newtons
    # gravity = 9.81
    # thrust_n = ...
    
    # 2. Use Linear Regression (y = mx + b)
    # y = thrust_n
    # x = pwm_values (or pwm_values squared)
    
    # TODO: Use np.polyfit(x, y, 1) to find the slope (Kt)
    
    return 0.0 # Placeholder
