"""
Standard Math Utilities for Robotics.
Follows The Squid Standard (Radians, Meters, Seconds).
"""

import numpy as np

def clamp(value, min_val, max_val):
    """Keep a value within a specific range."""
    return max(min(value, max_val), min_val)

def lerp(a, b, t):
    """Linear interpolation between a and b."""
    return a + t * (b - a)

def deadband(value, threshold):
    """Ignore small values (noise) near zero."""
    if abs(value) < threshold:
        return 0.0
    return value

def wrap_to_pi(angle):
    """Normalize an angle to the range [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def map_range(x, in_min, in_max, out_min, out_max):
    """Map a value from one range to another."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
