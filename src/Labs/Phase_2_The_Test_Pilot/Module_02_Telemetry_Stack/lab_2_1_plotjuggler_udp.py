SQUID DRONE CURRICULUM: lab_2_1_plotjuggler_udp.py
----------------------------------
Context: curriculum/Phases/Phase_2_The_Test_Pilot/Module_02_Telemetry_Stack/Module_02_Lecture.md
Goal: Stream data to PlotJuggler via UDP.

"""
Lab 2.1: The Data Stream
Objective: Send sensor data to PlotJuggler for real-time visualization.
"""

import socket
import time
import random

# CONFIG
LAPTOP_IP = "127.0.0.1" # Change this to your laptop's IP
PORT = 9870

# 1. Setup UDP Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def stream_data():
    print(f"Streaming to {LAPTOP_IP}:{PORT}... Press Ctrl+C to stop.")
    
    start_time = time.time()
    
    try:
        while True:
            elapsed = time.time() - start_time
            
            # Simulate Lidar data (Module 1 logic goes here)
            lidar_dist = 1.0 + random.uniform(-0.01, 0.01)
            
            # 2. Format as a CSV-like string for PlotJuggler
            # Format: "key1:val1,key2:val2"
            packet = f"time:{elapsed:.3f},lidar:{lidar_dist:.4f}"
            
            # 3. Send
            sock.sendto(packet.encode(), (LAPTOP_IP, PORT))
            
            time.sleep(0.02) # 50Hz
            
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    stream_data()
