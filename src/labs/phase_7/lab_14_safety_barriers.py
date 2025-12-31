"""
Lab 14.3: Control Barrier Functions (CBFs)
Goal: Implement a Safety Filter for wall avoidance.
Refer to docs/theory/Theory_14.3_Safety_Barriers.md
"""

def cbf_safety_filter(desired_velocity, distance_to_wall, alpha=0.5):
    """
    Inputs:
    desired_velocity: What the PID wants to do.
    distance_to_wall: Current Lidar reading.
    alpha: The 'stiffness' of the safety barrier.
    
    Output:
    safe_velocity: The modified command.
    """
    # h = distance_to_wall - buffer
    # safe_velocity must satisfy: h_dot >= -alpha * h
    
    # TODO: Calculate the safety constraint
    
    return desired_velocity # Placeholder
