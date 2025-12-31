"""
Module 2.3: The High-Frequency Logger
Goal: Capture every heartbeat of the drone for post-flight analysis.
"""

import csv
import time

class SquidLogger:
    def __init__(self, filename="flight_log.csv"):
        self.filename = filename
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        
        # TODO: Define your headers (e.g., timestamp, accel_x, gyro_y, motor_1)
        self.headers = ["timestamp", "ax", "ay", "az", "gx", "gy", "gz"]
        self.writer.writerow(self.headers)

    def log(self, data_dict):
        """
        data_dict: A dictionary containing the current state
        Example: {"ax": 0.1, "gx": 0.01, ...}
        """
        # TODO: Write the data row to the CSV
        pass

    def close(self):
        self.file.close()