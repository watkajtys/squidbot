"""
Lab 5.0: The PID Controller
Goal: Implement a stable PID loop for a single axis (e.g., Altitude).
Refer to docs/theory/Theory_4_PID_to_LQR.md
"""

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # TODO: Initialize memory variables for Integral and Derivative
        self.last_error = 0
        self.integral = 0
        self.last_time = None

    def update(self, setpoint, measurement, current_time):
        """
        Calculate the control output.
        setpoint: The value we want (e.g., 1.0 meter)
        measurement: The value we have (e.g., 0.8 meters)
        """
        # 1. Calculate Error
        # TODO: ...
        
        # 2. Calculate Delta Time (dt)
        # TODO: ...
        
        # 3. Proportional Term
        # P = self.kp * error
        
        # 4. Integral Term
        # TODO: self.integral += ...
        
        # 5. Derivative Term (Rate of Change)
        # TODO: derivative = ...
        
        # 6. Combined Output
        # output = P + I + D
        
        return 0.0 # Placeholder
