"""
Lab 5.9: Toss-and-Fly Recovery
Goal: Detect freefall and stabilize from an inverted state.
Refer to docs/theory/Theory_5.9_Recovery_from_Extreme_Attitudes.md
"""

import numpy as np

def detect_freefall(accel_vector):
    """Returns True if the drone is in freefall."""
    # TODO: Calculate magnitude of accel
    # return magnitude < 0.2
    return False

def get_recovery_torque(current_R, desired_R):
    """
    Calculates the shortest rotational path to level flight.
    """
    # 1. Calculate Rotation Error
    # error = 0.5 * (Rd.T @ R - R.T @ Rd)
    
    # TODO: Implement the matrix subtraction
    
    return np.array([0, 0, 0]) # Torque command [roll, pitch, yaw]
