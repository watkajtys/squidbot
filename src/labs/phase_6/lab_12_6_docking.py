"""
Lab 12.6: The Docking Pilot
Goal: Use an AprilTag to land on a small surface.
Refer to docs/theory/Theory_12.6_Autonomous_Docking_and_Recharging.md
"""

class DockingPilot:
    def __init__(self, tag_id=0):
        self.target_id = tag_id

    def get_control_command(self, tag_corners, image_center):
        """
        Inputs: 4 corners of the AprilTag found in the image.
        Outputs: [vx, vy, vz, yaw_rate]
        """
        # 1. Calculate error between tag_center and image_center
        # 2. Estimate Z-distance based on tag size
        # 3. Output a velocity command to center the tag
        
        # TODO: Implement visual servoing logic
        
        return [0, 0, 0, 0]
