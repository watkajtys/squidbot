
SQUID DRONE CURRICULUM: lab_4_2_ekf.py
----------------------------------
Context: ../../../../curriculum/Phases/Phase_4_The_Architect/Module_07_State_Estimation/Module_07_Lecture.md
Theory: curriculum/Library/
Win Condition: Phase IV: The Paper Trick (SQUID_GAMES.md)

This lab is part of the 'Soldering Iron to Swarm' masterclass.

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
        # PhD Tip: Don't forget to subtract gravity if your IMU is NED centered!
        # F = [[1, dt], [0, 1]]
        
        # TODO: Implement Prediction Math
        # self.x = F @ self.x + B @ u
        # self.P = F @ self.P @ F.T + Q (Process Noise)
        pass

    def update(self, lidar_dist):
        """
        Step B: Sensor-based correction.
        lidar_dist: Measured distance to floor
        """
        # PhD Tip: Use the Joseph Form for numerical stability if self.P drifts.
        # H = [[1, 0]]
        
        # TODO: Calculate Innovation (y = z - Hx)
        # TODO: Calculate Kalman Gain (K = P H^T S^-1)
        # TODO: Update State (x = x + Ky)
        # TODO: Update Covariance (P = (I - KH)P)
        pass
