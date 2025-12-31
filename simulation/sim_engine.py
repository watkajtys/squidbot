"""
Squid Drone SITL (Software-In-The-Loop) Engine
A lightweight PyBullet wrapper for testing labs without hardware.
"""

import pybullet as p
import pybullet_data
import time
import numpy as np

class SquidSim:
    def __init__(self, render=True):
        self.physics_client = p.connect(p.GUI if render else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load Ground
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Load Squid Drone (URDF)
        # Note: quadrotor.urdf must exist in the simulation/ directory
        try:
            self.drone_id = p.loadURDF("simulation/quadrotor.urdf", [0, 0, 0.1])
        except:
            print("Warning: simulation/quadrotor.urdf not found. Using generic sphere.")
            self.drone_id = p.loadURDF("sphere_1cm.urdf", [0, 0, 0.1])

    def get_state(self):
        """Returns [pos, quat, lin_vel, ang_vel]"""
        pos, quat = p.getBasePositionAndOrientation(self.drone_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.drone_id)
        return np.array(pos), np.array(quat), np.array(lin_vel), np.array(ang_vel)

    def apply_thrusts(self, motor_thrusts):
        """
        Applies forces to the 4 motor locations.
        motor_thrusts: [M1, M2, M3, M4] in Newtons.
        """
        # TODO: Map these thrusts to the URDF link indices
        # For now, apply a simple central force for testing
        total_thrust = sum(motor_thrusts)
        p.applyExternalForce(self.drone_id, -1, [0, 0, total_thrust], [0, 0, 0], p.LINK_FRAME)

    def step(self):
        p.stepSimulation()
        time.sleep(1./240.)

if __name__ == "__main__":
    sim = SquidSim()
    for _ in range(1000):
        # Simulate a hover thrust
        sim.apply_thrusts([0.5, 0.5, 0.5, 0.5]) 
        sim.step()
