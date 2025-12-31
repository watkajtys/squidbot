
SQUID DRONE CURRICULUM: lab_0_3_motor_safety.py
----------------------------------
Context: ../../../../curriculum/Phases/Phase_1_The_Mechanic/Module_00_The_Build/Module_00_Lecture.md
Theory: curriculum/Library/
Win Condition: Phase I: The Morse Code (SQUID_GAMES.md)

This lab is part of the 'Soldering Iron to Swarm' masterclass.

"""
Lab 0.3: Motor Safety Test (PROPS OFF)
Goal: Use the MSP driver to spin motors on the bench.
Refer to curriculum/phase_1_mechanic/Module_0_Labs.md

CRITICAL: REMOVE ALL PROPELLERS BEFORE RUNNING THIS SCRIPT.
"""

from src.drivers.msp import MSPDriver # Assuming this exists
import time

def safety_spin_test():
    # TODO: Initialize the MSP driver on /dev/ttyAMA0 (or similar)
    # TODO: Send a 'Motor Arm' command (if required by your safety firmware)
    # TODO: Gradually increase the thrust of Motor 1 to 10%
    # TODO: Stop all motors
    pass

if __name__ == "__main__":
    print("WARNING: ENSURE PROPELLERS ARE REMOVED.")
    input("Press Enter to continue...")
    safety_spin_test()
