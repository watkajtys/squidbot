#!/usr/bin/env python3
"""
Squid Drone Forensic Regression Suite
Module 07.5: Verify code changes against 'Golden' flight data.

Usage:
    python3 regression_test.py --bag golden_flight.mcap --output results.csv
"""

import argparse
import csv
import json
import numpy as np
import os
import sys

# Mocking a simple EKF or Controller for the lab
# In a real scenario, the student would import their actual classes from src/
class MockEKF:
    def __init__(self, accel_noise=0.1):
        self.state = 0.0
        self.accel_noise = accel_noise
        self.variance = 1.0
        self.last_t = None

    def update(self, t, accel_z, baro_alt):
        if self.last_t is None:
            self.last_t = t
            self.state = baro_alt
            return self.state

        dt = t - self.last_t
        self.last_t = t

        # Prediction step (Simple Constant Velocity)
        # state = state + 0.5 * accel * dt^2 (simplified)
        self.state += 0.5 * accel_z * (dt**2)
        
        # Update step (Simple Kalman Gain)
        innovation = baro_alt - self.state
        gain = 0.1 # Static gain for the mock
        self.state += gain * innovation
        
        return self.state

def run_regression(bag_path, modified_param=None):
    """
    Simulates replaying data from a 'Golden Bag'.
    Since we don't have a real .mcap parser here without dependencies,
    we'll simulate the data stream for the purpose of the lab template.
    """
    print(f"[*] Loading Golden Bag: {bag_path}")
    
    # In reality, students would use mcap.reader or rosbag2_py
    # Here we generate a 'Golden' dataset
    t_series = np.linspace(0, 10, 100)
    golden_alt = np.sin(t_series) + 5.0 # A nice smooth flight
    raw_accel = np.cos(t_series) * 0.1 + np.random.normal(0, 0.05, 100)
    raw_baro = golden_alt + np.random.normal(0, 0.02, 100)

    # 1. Run the "Reference" (Original Code)
    ref_ekf = MockEKF(accel_noise=0.1)
    ref_results = []
    for i in range(len(t_series)):
        est = ref_ekf.update(t_series[i], raw_accel[i], raw_baro[i])
        ref_results.append(est)

    # 2. Run the "Modified" (New Code)
    # We simulate a "regression" by changing the noise parameter
    new_noise = modified_param if modified_param is not None else 0.5
    new_ekf = MockEKF(accel_noise=new_noise)
    new_results = []
    for i in range(len(t_series)):
        est = new_ekf.update(t_series[i], raw_accel[i], raw_baro[i])
        new_results.append(est)

    # 3. Calculate Metrics
    rmse = np.sqrt(np.mean((np.array(new_results) - np.array(ref_results))**2))
    
    print("-" * 30)
    print(f"REGRESSION RESULTS")
    print(f"Reference Accel Noise: 0.1")
    print(f"Modified Accel Noise:  {new_noise}")
    print(f"Calculated RMSE:       {rmse:.6f} meters")
    print("-" * 30)

    if rmse > 0.01:
        print("[FAIL] Regression detected! RMSE > 1cm.")
    else:
        print("[PASS] No significant regression.")

    return t_series, ref_results, new_results, rmse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Squid Regression Suite")
    parser.add_argument("--bag", type=str, help="Path to the .mcap flight bag", default="data/golden_bag.mcap")
    parser.add_argument("--param", type=float, help="Simulated param change for the lab", default=0.1)
    
    args = parser.parse_args()

    # Create dummy data dir if it doesn't exist
    if not os.path.exists('data'):
        os.makedirs('data')

    run_regression(args.bag, args.param)
