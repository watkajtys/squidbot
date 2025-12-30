import time
import csv

class BlackBox:
    def __init__(self, filename="flight_log.csv"):
        self.filename = filename
        self.buffer = []
        # TODO: Write CSV Header
        pass

    def log(self, timestamp, raw_dist, motor_out):
        """
        Add a data point to memory.
        """
        self.buffer.append([timestamp, raw_dist, motor_out])

    def save(self):
        """
        Dump memory to disk.
        """
        # TODO: Write self.buffer to self.filename
        print(f"Log saved to {self.filename}")
