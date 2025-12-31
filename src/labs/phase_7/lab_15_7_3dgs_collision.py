"""
Lab 15.7: 3D Gaussian Splatting (3DGS) Collision Check
Goal: Use differentiable ellipsoids for sub-centimeter obstacle avoidance.
Refer to docs/study_guides/Module_15_Study_Guide.md
"""

import numpy as np

class GSMap:
    def __init__(self):
        # A list of Gaussians: {mu (center), Sigma (shape), opacity}
        self.gaussians = []

    def load_map(self, file_path):
        """Load a .ply or .json 3DGS model."""
        pass

    def calculate_collision_probability(self, drone_pos, drone_radius):
        """
        Mathematically intersect the drone's sphere with the 3DGS ellipsoids.
        This is differentiable!
        """
        # TODO: Sum the opacity of Gaussians weighted by their distance
        # prob = sum( alpha_i * exp(-0.5 * (x-mu)^T * Sigma^-1 * (x-mu)) )
        return 0.0

    def get_gradient(self, drone_pos):
        """
        Return the direction that most quickly reduces collision probability.
        (The 'Safe' direction).
        """
        # TODO: Calculate d(Prob)/d(pos)
        return np.array([0, 0, 0])
