"""
Lab 3.5: SITL Driver Bridge
Goal: Connect your production code to the PyBullet simulator.
Refer to docs/simulation/README.md
"""

class SITLBridge:
    def __init__(self):
        # TODO: Initialize sim_engine.SquidSim
        pass

    def send_to_sim(self, msp_packet):
        """Translate MSP motor commands to PyBullet forces."""
        pass

    def get_from_sim(self):
        """Translate PyBullet state to mock IMU/ToF data."""
        pass
