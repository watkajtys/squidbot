"""
Squid Drone: Zero Hour Simulation
"Fly before you build."

This script launches a physics simulation of the drone using PyBullet.
Controls:
    Up Arrow:   Pitch Forward
    Down Arrow: Pitch Backward
    Left Arrow: Roll Left
    Right Arrow: Roll Right
    W:          Throttle Up
    S:          Throttle Down
    A:          Yaw Left
    D:          Yaw Right
"""

import time
import numpy as np
import pybullet as p
import pybullet_data
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl

def run_sim():
    # 1. Setup Environment
    env = CtrlAviary(drone_model=DroneModel.CF2X, num_drones=1, physics=Physics.PYB, gui=True)
    
    # 2. Setup Controller (PID)
    ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)
    
    # 3. Initial State
    action = {'0': np.array([0, 0, 0, 0])}
    target_pos = np.array([0, 0, 1.0]) # Hover at 1m
    target_rpy = np.array([0, 0, 0])
    
    print("[INFO] Simulation Started. Click the window to focus.")
    print("[INFO] Use Arrow Keys to nudge Position. W/S for Altitude.")

    # 4. Simulation Loop
    START = time.time()
    try:
        while True:
            # Sync with real time
            t = time.time() - START
            
            # Simple "Keyboard" Logic (Mocking inputs for demo)
            # In a real app, we'd capture p.getKeyboardEvents()
            keys = p.getKeyboardEvents()
            
            # Default: Hover
            yaw_cmd = 0.0
            
            # Parse Keys
            if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
                target_pos[0] += 0.05
            if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
                target_pos[0] -= 0.05
            if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
                target_pos[1] += 0.05
            if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
                target_pos[1] -= 0.05
            if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
                target_pos[2] += 0.05
            if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
                target_pos[2] -= 0.05
            if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
                yaw_cmd = 0.1
            if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
                yaw_cmd = -0.1

            # Get State
            obs = env._getObservationForDrone(0) # Pos, Quat, RPY, Vel, AngVel, RPM
            current_pos = obs[0:3]
            current_rpy = obs[7:10]
            
            # Compute Control (PID)
            rpm, _, _ = ctrl.computeControl(control_timestep=0.02,
                                            cur_pos=current_pos,
                                            cur_quat=obs[3:7],
                                            cur_vel=obs[10:13],
                                            cur_ang_vel=obs[13:16],
                                            target_pos=target_pos,
                                            target_rpy=np.array([0,0,0]), # Simplified
                                            target_vel=np.zeros(3),
                                            target_rpy_rates=np.zeros(3)
                                            )
            
            # Step Physics
            env.step({'0': rpm})
            
            # Render
            p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=-30, cameraPitch=-30, cameraTargetPosition=current_pos)
            time.sleep(1/240.0) # Real-time sync

    except KeyboardInterrupt:
        env.close()

if __name__ == "__main__":
    run_sim()
