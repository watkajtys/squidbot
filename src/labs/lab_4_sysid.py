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
