
SQUID DRONE CURRICULUM: lab_7_3_rvo_collision.py
----------------------------------
Context: ../../../../curriculum/Phases/Phase_7_The_Frontier/Module_14_Swarm_Theory/Module_14_Lecture.md
Theory: curriculum/Library/
Win Condition: Phase VII: Consensus Dance (SQUID_GAMES.md)

This lab is part of the 'Soldering Iron to Swarm' masterclass.

"""
Lab 14.5: RVO (Reciprocal Velocity Obstacles)
Goal: Avoid dynamic obstacles without a central controller.

THE "AI ENGINEER" GUIDE:
------------------------
This is Geometry, not Magic. We are just checking if our velocity vector points
inside a "Danger Circle."
Ask an AI:
"I have two 2D points and velocities. Calculate the 'Time to Closest Approach' (tau).
If tau is positive and small, calculate a velocity correction vector to avoid collision."
"""

import numpy as np

def calculate_rvo_velocity(current_pos, current_vel, pref_vel, obstacle_pos, obstacle_vel, radius=0.5):
    """
    Calculates a safe velocity to avoid a moving obstacle.
    Using a simplified 'Velocity Obstacle' (VO) logic.
    """
    # 1. Relative State
    # "The world from my perspective"
    rel_pos = obstacle_pos - current_pos
    rel_vel = current_vel - obstacle_vel
    
    dist_sq = np.dot(rel_pos, rel_pos)
    dist = np.sqrt(dist_sq)
    
    # 2. Are we too close already?
    min_dist = radius * 2.0 # My radius + Their radius
    if dist < min_dist:
        # EMERGENCY: Move directly away!
        return -1.0 * (rel_pos / dist)
        
    # 3. Collision Cone Geometry
    # Will we hit them if we keep this velocity?
    # We project our relative velocity onto the relative position vector.
    # Dot Product > 0 means we are moving TOWARDS them.
    # Dot Product < 0 means we are moving AWAY.
    vel_proj = np.dot(rel_vel, rel_pos)
    
    # If moving away, we are safe.
    if vel_proj < 0:
        return pref_vel
        
    # 4. Check 'Time to Collision' (Simple Raycast)
    # If we are moving towards them, how close will we get?
    # Vector Rejection logic
    unit_rel_pos = rel_pos / dist
    vel_parallel = np.dot(rel_vel, unit_rel_pos) * unit_rel_pos
    vel_perp = rel_vel - vel_parallel
    miss_distance = np.linalg.norm(vel_perp)
    
    # If our trajectory misses them by a wide margin, we are safe.
    if miss_distance > min_dist:
        return pref_vel
        
    # 5. The Avoidance Maneuver (The "Nudge")
    # We are on a collision course. We need to change velocity.
    # RVO says: "I change half, you change half."
    
    # Calculate the normal vector (perpendicular to position)
    # This is the direction we should "slide" to avoid them
    tangent = np.array([-unit_rel_pos[1], unit_rel_pos[0]])
    
    # Pick the direction closer to our preferred velocity
    if np.dot(pref_vel, tangent) < 0:
        tangent = -tangent
        
    # New Velocity = Average of (Current, Preferred) + Nudge
    # This is a simplified RVO behavior
    avoidance_strength = 2.0
    safe_vel = 0.5 * (current_vel + pref_vel) + (tangent * avoidance_strength)
    
    # Normalize to max speed (e.g., 1.0 m/s)
    speed = np.linalg.norm(safe_vel)
    if speed > 1.0:
        safe_vel = safe_vel / speed
        
    return safe_vel
