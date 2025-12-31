
SQUID DRONE CURRICULUM: lab_6_4_search_heatmap.py
----------------------------------
Context: ../../../../curriculum/Phases/Phase_6_The_Specialist/Module_11_Tactics_and_Guidance/Module_11_Lecture.md
Theory: curriculum/Library/
Win Condition: Phase VI: Fox Two Lock-on (SQUID_GAMES.md)

This lab is part of the 'Soldering Iron to Swarm' masterclass.

"""
Lab 11.5: Bayesian Search Heatmap
Goal: Maintain a probability map of a target's location.
Refer to docs/theory/Theory_11.5_Bayesian_Search_Theory.md
"""

import numpy as np

class SearchHeatmap:
    def __init__(self, room_size=(10, 10)):
        # TODO: Initialize grid with uniform probability
        self.grid = np.full(room_size, 1.0 / (room_size[0] * room_size[1]))

    def update_cell(self, x, y, pd=0.9):
        """
        Update the heatmap when we look at cell (x, y) and see nothing.
        pd: Probability of Detection (How much we trust our camera).
        """
        # 1. Multiply the cell by (1 - pd)
        # TODO: self.grid[x, y] *= ...
        
        # 2. Normalize the grid so the total sum is 1.0 again
        # TODO: self.grid /= np.sum(self.grid)
        pass

    def get_target_guess(self):
        """Returns the cell with the highest probability."""
        return np.unravel_index(np.argmax(self.grid), self.grid.shape)
