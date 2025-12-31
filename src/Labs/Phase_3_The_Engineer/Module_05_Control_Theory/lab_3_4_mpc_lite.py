
SQUID DRONE CURRICULUM: lab_3_4_mpc_lite.py
----------------------------------
Context: ../../../../curriculum/Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md
Theory: curriculum/Library/
Win Condition: Phase III: The Statue Hover (SQUID_GAMES.md)

This lab is part of the 'Soldering Iron to Swarm' masterclass.

"""
Lab 5.8: Linear MPC Lite
Goal: Predict and follow a trajectory while respecting constraints.
Refer to docs/theory/Theory_5.8_Model_Predictive_Control.md
"""

import numpy as np

class MPCLite:
    def __init__(self, horizon=10, dt=0.1):
        self.N = horizon # Number of steps to look ahead
        self.dt = dt
        
        # TODO: Define your A and B matrices (Physics)
        # x_{k+1} = A*x_k + B*u_k

    def solve(self, current_state, goal_state):
        """
        Finds the sequence of 'u' (motor commands) that minimizes
        the error to the goal over the horizon N.
        """
        # 1. Cost Function: sum(error^2 + input^2)
        # 2. Constraints: u_min <= u <= u_max
        
        # TODO: Use np.linalg.lstsq or a simple optimizer
        
        # Return only the first command
        return 0.0 
