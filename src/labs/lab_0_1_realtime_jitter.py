"""
Lab 0.1: Real-Time Jitter Watchdog
Goal: Measure the 'Determinism' of your Python control loop.
Refer to docs/theory/Theory_0.1_The_Real_Time_Budget.md

If your 20ms loop occasionally takes 30ms, the PID math will become unstable.
"""

import time
import numpy as np

class JitterWatchdog:
    def __init__(self, target_dt=0.02):
        self.target_dt = target_dt
        self.history = []
        self.last_time = time.perf_counter()

    def tick(self):
        """Measure the time since the last tick."""
        now = time.perf_counter()
        actual_dt = now - self.last_time
        self.history.append(actual_dt)
        self.last_time = now

    def analyze_jitter(self):
        """
        Calculate the standard deviation and max latency.
        """
        if not self.history: return
        
        dts = np.array(self.history)
        # TODO: Calculate mean, std_dev, and max_jitter
        # Jitter = abs(actual_dt - target_dt)
        pass
