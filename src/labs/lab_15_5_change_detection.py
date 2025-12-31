"""
Lab 15.5: Spatio-Temporal Change Detection
Goal: Detect if a room has been altered since the last map was built.
Refer to docs/theory/Theory_15.5_Spatio_Temporal_Change_Detection.md
"""

import numpy as np

class ChangeDetector:
    def __init__(self, baseline_map):
        self.baseline = baseline_map

    def compare_frames(self, current_frame):
        """
        Identify 'New' voxels that aren't in the baseline.
        Useful for detecting moving obstacles or structural collapses.
        """
        # TODO: Implement a probabilistic diff
        pass
