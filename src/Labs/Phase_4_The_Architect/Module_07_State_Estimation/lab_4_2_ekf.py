
SQUID DRONE CURRICULUM: lab_4_2_ekf.py
----------------------------------
Context: ../../../../curriculum/Phases/Phase_4_The_Architect/Module_07_State_Estimation/Module_07_Lecture.md
Theory: curriculum/Library/
Win Condition: Phase IV: The Paper Trick (SQUID_GAMES.md)

This lab is part of the 'Soldering Iron to Swarm' masterclass.

"""
Lab 7.0: The Extended Kalman Filter (EKF)
Goal: Fuse noisy Sensors (Lidar) with fast Sensors (IMU) to find the truth.

THE "AI ENGINEER" GUIDE:
------------------------
The EKF is just a weighted average between "Prediction" (Physics) and "Correction" (Sensors).
If this breaks, ask an AI:
"My EKF covariance matrix P became non-positive definite. 
Check my 'Joseph Form' implementation in the update step."
"""

import numpy as np

class AltitudeEKF:
    def __init__(self, dt=0.02):
        self.dt = dt
        
        # State Vector: [Altitude (z), Velocity (vz)]
        self.x = np.zeros((2, 1))
        
        # Covariance Matrix (P): How confident are we?
        # Diagonals: Variance of Z, Variance of Vz
        # High value = "I have no idea where I am"
        self.P = np.array([[10.0, 0.0],
                           [0.0, 10.0]])
        
        # Process Noise (Q): Uncertainty in our Physics Model
        # "Wind gusts, motor jitters"
        self.Q = np.array([[0.01, 0.0],
                           [0.0, 0.1]])

        # Measurement Noise (R): Uncertainty in our Lidar
        # "Lidar is accurate to +/- 5cm"
        self.R = np.array([[0.05]]) 

        # State Transition Matrix (F) - The Physics Engine
        # z_new = z + vz * dt
        # vz_new = vz
        self.F = np.array([[1, dt],
                           [0, 1]])

        # Control Input Matrix (B) - The Motor's Effect
        # z += 0.5 * accel * dt^2
        # vz += accel * dt
        self.B = np.array([[0.5 * dt**2],
                           [dt]])

    def predict(self, accel_z):
        """
        Step A: PREDICT (The "Blind" Guess)
        We close our eyes and guess where we are based on how much we throttled.
        """
        # 1. Project State Ahead (x = Fx + Bu)
        # We subtract 9.81 because the accelerometer measures gravity even when stopped
        net_accel = accel_z - 9.81 
        u = np.array([[net_accel]])
        
        self.x = (self.F @ self.x) + (self.B @ u)

        # 2. Project Uncertainty Ahead (P = F P F.T + Q)
        # "Uncertainty grows over time"
        self.P = (self.F @ self.P @ self.F.T) + self.Q

    def update(self, lidar_dist):
        """
        Step B: UPDATE (The "Reality" Check)
        We open our eyes (Lidar) and correct our guess.
        """
        # Measurement Mapping (H): Extract just 'Position' from state [Pos, Vel]
        H = np.array([[1, 0]])
        
        # 1. The Innovation (y): "Surprise" = Measurement - Expected
        z_measured = lidar_dist
        z_expected = H @ self.x
        y = z_measured - z_expected
        
        # 2. Innovation Covariance (S): Total Uncertainty (Estimate + Sensor)
        S = (H @ self.P @ H.T) + self.R
        
        # 3. Kalman Gain (K): The "Trust Factor"
        # If Sensor is clean (R is small), K is high -> Trust Sensor
        # If Physics is good (P is small), K is low  -> Trust Physics
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # 4. Update State (x = x + Ky)
        self.x = self.x + (K @ y)
        
        # 5. Update Uncertainty (P) - The Joseph Form (PhD Grade Stability)
        # Simple Form: P = (I - KH)P  <-- Unstable! Can cause crashes.
        # Joseph Form: P = (I-KH)P(I-KH)' + KRK' <-- Guaranteed Stable.
        I = np.eye(2)
        ImKH = I - (K @ H)
        self.P = (ImKH @ self.P @ ImKH.T) + (K @ self.R @ K.T)
        
        return self.x[0,0] # Return estimated altitude
