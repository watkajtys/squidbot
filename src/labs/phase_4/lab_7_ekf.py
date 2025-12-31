"""
Lab 7.0: The Extended Kalman Filter (EKF)
Goal: Fuse IMU and Lidar data for Altitude estimation.
Refer to docs/theory/Theory_2_The_EKF.md
"""

import numpy as np

class AltitudeEKF:
    def __init__(self):
        # TODO: Initialize State Vector [Altitude, Velocity]
        # x = [0, 0]
        self.x = np.zeros((2, 1))
        
        # TODO: Initialize Covariance Matrix (P)
        # How uncertain are we at the start?
        self.P = np.eye(2) * 1.0

    def predict(self, accel_z, dt):
        """
        Step A: Physics-based prediction.
        accel_z: Acceleration in World Frame (m/s^2)
        """
        # TODO: Implement the State Transition Matrix (F)
        # F = [[1, dt], [0, 1]]
        
        # TODO: Implement Prediction Math
        # self.x = F @ self.x + ...
        # self.P = F @ self.P @ F.T + Q
        pass

    def update(self, lidar_dist):
        """
        Step B: Sensor-based correction.
        lidar_dist: Measured distance to floor
        """
        # TODO: Implement Measurement Matrix (H)
        # H = [[1, 0]]
        
        # TODO: Calculate Kalman Gain (K)
        # TODO: Update State (x)
        # TODO: Update Covariance (P)
        pass
