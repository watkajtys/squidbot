"""
Lab 10.6: Domain Randomization
Goal: Randomly vary physics parameters to train robust AI.
Refer to docs/theory/Theory_10.6_Sim_to_Real_Transfer.md
"""

import random

class RandomizedDroneEnv:
    def __init__(self):
        # The 'True' center values from SysID
        self.base_mass = 0.050 # 50 grams
        self.base_kt = 0.15

    def reset(self):
        """Called at the start of every training episode."""
        # 1. Randomize Mass (+/- 10%)
        # TODO: self.current_mass = ...
        
        # 2. Randomize Thrust Constant
        # TODO: self.current_kt = ...
        
        # 3. Apply to the physics engine (PyBullet)
        pass
