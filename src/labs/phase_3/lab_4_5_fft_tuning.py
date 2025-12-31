"""
Lab 4.5: Resonance and Notch Filtering
Goal: Use Fast Fourier Transforms (FFT) on IMU data to identify frame resonance.
Refer to docs/theory/Theory_0.8_High_Speed_Aerodynamics.md

This lab is required to prevent the drone from vibrating itself to death at high throttle.
"""

import numpy as np

class VibrationAnalyzer:
    def __init__(self, sampling_rate=500):
        self.fs = sampling_rate
        self.buffer = []

    def record_sample(self, gyro_z):
        self.buffer.append(gyro_z)

    def find_resonance_peak(self):
        """
        Perform an FFT and return the frequency (Hz) with the highest energy.
        """
        if len(self.buffer) < 256: return 0
        
        # TODO: Implement np.fft.fft logic
        # 1. Apply a Hanning window
        # 2. Calculate the magnitude spectrum
        # 3. Identify the peak between 40Hz and 200Hz
        return 0.0

    def calculate_notch_filter_params(self, center_freq):
        """
        Calculate Alpha/Beta for a digital notch filter.
        """
        # TODO: Convert frequency to digital filter coefficients
        pass
