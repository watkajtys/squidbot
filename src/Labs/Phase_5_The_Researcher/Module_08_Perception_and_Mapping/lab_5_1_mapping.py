
SQUID DRONE CURRICULUM: lab_5_1_mapping.py
----------------------------------
Context: ../../../../curriculum/Phases/Phase_5_The_Researcher/Module_08_Perception_and_Mapping/Module_08_Lecture.md
Theory: curriculum/Library/
Win Condition: Phase V: The Mirror Maze (SQUID_GAMES.md)

This lab is part of the 'Soldering Iron to Swarm' masterclass.

"""
Lab 8.6: The Perch Finder (RANSAC)
Goal: Analyze Lidar data to find the highest flat surface.

THE "AI ENGINEER" GUIDE:
------------------------
RANSAC (Random Sample Consensus) is just "Guess and Check."
1. Pick 3 random points.
2. Assume they form a plane.
3. Count how many other points fit that plane.
4. Repeat 100 times. Keep the best one.

Ask an AI:
"I have a numpy array of 3D points. Write a RANSAC loop to find the equation (ax + by + cz + d = 0) of the largest horizontal plane."
"""

import numpy as np

def fit_plane_ransac(points, iterations=100, threshold=0.01):
    """
    Finds the best plane model for a point cloud.
    Returns: (a, b, c, d), inliers
    """
    best_plane = None
    best_count = 0
    
    n_points = points.shape[0]
    
    for _ in range(iterations):
        # 1. Pick 3 random points
        idx = np.random.choice(n_points, 3, replace=False)
        p1, p2, p3 = points[idx]
        
        # 2. Compute Plane Normal (Cross Product)
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        norm_len = np.linalg.norm(normal)
        
        if norm_len == 0: continue # Collinear points
        
        normal = normal / norm_len
        a, b, c = normal
        d = -np.dot(normal, p1)
        
        # 3. Count Inliers (Distance from point to plane)
        # Dist = |ax + by + cz + d|
        distances = np.abs(np.dot(points, normal) + d)
        count = np.sum(distances < threshold)
        
        # 4. Keep Best
        if count > best_count:
            best_count = count
            best_plane = (a, b, c, d)
            
    return best_plane

def find_highest_plane(point_cloud):
    """
    Inputs: Nx3 array of [x, y, z] points.
    Outputs: [x, y, z] coordinates of the best perch center.
    """
    # 1. Filter: Keep only points where Z > 1.0 (Exclude floor)
    high_points = point_cloud[point_cloud[:, 2] > 1.0]
    
    if len(high_points) < 10:
        return None
        
    # 2. Run RANSAC
    plane_model = fit_plane_ransac(high_points)
    
    if plane_model:
        # Calculate centroid of the perch
        # (Simplified: Just mean of high points)
        center = np.mean(high_points, axis=0)
        return center
        
    return np.array([0, 0, 0])
