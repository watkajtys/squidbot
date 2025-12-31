"""
The Squid Data Simulator
Goal: Generate synthetic "noisy" data to test your FFT and Filter code.
"""

import numpy as np
import matplotlib.pyplot as plt

def generate_vibration_data(duration=1.0, sample_rate=1000, noise_hz=333, noise_amp=0.5):
    """
    Simulates drone gyro data with a specific motor vibration.
    """
    t = np.linspace(0, duration, int(sample_rate * duration))
    
    # The "True" signal (Drone slowly tilting)
    true_signal = np.sin(2 * np.pi * 1.0 * t) 
    
    # The "Motor Noise" (High frequency vibration)
    vibration = noise_amp * np.sin(2 * np.pi * noise_hz * t)
    
    # Add some random white noise (Sensor jitter)
    white_noise = np.random.normal(0, 0.1, len(t))
    
    return t, true_signal + vibration + white_noise

if __name__ == "__main__":
    print("Generating sample vibration data (333Hz)...")
    t, data = generate_vibration_data()
    
    # In Module 4, you will write the code to find the 333Hz spike in this data!
    plt.plot(t[:100], data[:100])
    plt.title("Synthetic Drone Data (First 100ms)")
    plt.xlabel("Time (s)")
    plt.ylabel("Gyro Reading")
    plt.show()
    print("Done. Use this data to test your FFT and Notch filters.")
 Lands.
