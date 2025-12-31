"""
Lab 13.0: The Visual Front-End (KLT Tracker)
Goal: Track "Pixels of Interest" across multiple camera frames.
Refer to docs/Module_13_Visual_Inertial_Odometry.md
"""

import cv2
import numpy as np

class FeatureTracker:
    def __init__(self):
        # TODO: Setup ORB, SIFT, or KLT parameters
        pass

    def track(self, prev_frame, curr_frame, prev_points):
        """
        Uses OpenCV to track points from one image to the next.
        """
        # TODO: Implement cv2.calcOpticalFlowPyrLK()
        # returns: new_points, status, error
        pass

    def find_new_features(self, frame, existing_points):
        """
        If we lose points, find new 'strong' corners to track.
        """
        # TODO: Implement cv2.goodFeaturesToTrack()
        pass
