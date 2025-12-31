"""
The Squid Transform Registry (Static TF)
Goal: Define the exact 3D position of every sensor relative to the Center of Mass.

Follows REP-105 standards.
Units: Meters
Frame: [X (Forward), Y (Left), Z (Up)]
"""

import numpy as np

# --- PHYSICAL CALIBRATION (Change these for your specific build!) ---

# Position of the Lidar sensor relative to the Center of Mass
# Example: 1cm forward, 0cm left/right, 2cm below CoM
LIDAR_DOWN_OFFSET = np.array([0.01, 0.00, -0.02])

# Position of the Camera relative to the Center of Mass
CAMERA_OFFSET = np.array([0.03, 0.00, 0.01])

# --- UTILITIES ---

def get_lidar_world_pos(drone_pos, drone_orientation_quat):
    """
    Translates the raw Lidar reading into a World-Frame coordinate
    using the Static Transform.
    """
    # TODO: Implement the rotation of the offset vector by the drone's orientation
    # and add it to the drone's position.
    pass
