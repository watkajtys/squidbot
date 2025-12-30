import smbus2
import RPi.GPIO as GPIO
import time

class ToFArray:
    def __init__(self):
        """
        Initialize the two VL53L1X sensors.
        """
        self.bus = smbus2.SMBus(1)
        self.FRONT_XSHUT = 17
        self.DOWN_XSHUT = 27
        
        # TODO: Setup GPIO mode
        # TODO: Call self._reset_sensors()
        pass

    def _reset_sensors(self):
        """
        The 'Boot Sequence' to fix the address conflict.
        1. Turn both OFF (XSHUT Low)
        2. Turn Front ON -> Change Address to 0x30
        3. Turn Down ON -> Keep Address 0x29
        """
        pass

    def read_distances(self):
        """
        Returns tuple: (front_dist_mm, down_dist_mm)
        """
        # TODO: Read from both I2C addresses
        return (0, 0)
