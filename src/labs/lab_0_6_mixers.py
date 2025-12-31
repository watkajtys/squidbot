"""
Lab 0.6: The Motor Mixer
Goal: Convert Thrust/Roll/Pitch/Yaw into 4 Motor Speeds.
Refer to docs/theory/Theory_0.6_The_Motor_Mixer_Matrix.md
"""

def motor_mixer(thrust, roll, pitch, yaw):
    """
    Inputs range from -1.0 to 1.0 (except thrust, which is 0.0 to 1.0).
    Outputs should be 0.0 to 1.0 for each motor.
    """
    # 1. Implement the Mix Equations
    m1 = thrust + roll + pitch - yaw
    m2 = thrust - roll + pitch + yaw
    m3 = thrust - roll - pitch - yaw
    m4 = thrust + roll - pitch + yaw
    
    # 2. Handle Saturation (The 'Elite' part)
    # TODO: If any motor > 1.0, find the max and scale all motors down.
    # TODO: If any motor < 0.0, clip it to 0.0.
    
    return [m1, m2, m3, m4]
