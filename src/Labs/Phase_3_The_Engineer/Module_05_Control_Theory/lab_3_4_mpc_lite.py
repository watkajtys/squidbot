
SQUID DRONE CURRICULUM: lab_3_4_mpc_lite.py
----------------------------------
Context: ../../../../curriculum/Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md
Theory: curriculum/Library/
Win Condition: Phase III: The Statue Hover (SQUID_GAMES.md)

This lab is part of the 'Soldering Iron to Swarm' masterclass.

"""
Lab 5.8: Linear MPC Lite (The "Batch" Approach)
Goal: Predict the future to control the present.

THE "AI ENGINEER" GUIDE:
------------------------
You don't need to derive these matrices by hand. If you get stuck, paste this code 
into an AI and ask:
"Explain how the 'Batch Matrices' (S_bar and T_bar) allow us to solve for 
all future motor commands in a single step using Least Squares."
"""

import numpy as np

class MPCLite:
    def __init__(self, horizon=10, dt=0.1):
        self.N = horizon  # Look ahead 10 steps (e.g., 1.0 second)
        self.dt = dt
        
        # --- THE PHYSICS (Double Integrator Model) ---
        # "Where will I be?" (Position += Velocity * dt)
        # "How fast will I be?" (Velocity += Acceleration * dt)
        # State x = [Position, Velocity]
        self.A = np.array([[1, dt],
                           [0, 1]])
        
        # "How much does the motor affect me?"
        # Position += 0.5 * accel * dt^2
        # Velocity += accel * dt
        self.B = np.array([[0.5 * dt**2],
                           [dt]])
        
        # --- THE COST (Tuning) ---
        # Q: How much we hate error (High = Aggressive, Low = Smooth)
        self.Q = np.diag([10.0, 1.0]) # Hate Position error (10x), Velocity error (1x)
        self.R = np.diag([0.1])       # Don't waste battery (Penalty on high motor thrust)

    def solve(self, current_state, goal_state):
        """
        Solves for the optimal motor commands for the next N steps.
        Returns: The immediate motor command (acceleration) to apply.
        """
        # 1. Setup the "Batch" matrices
        # This converts the recursive loop (x_k+1 = Ax + Bu) into one big matrix equation:
        # X_future = T * x_current + S * U_future
        T, S = self._build_batch_matrices()
        
        # 2. Build the Reference Trajectory (Where we want to be)
        # We want to be at 'goal_state' for all N steps
        X_ref = np.tile(goal_state, (self.N, 1)).flatten()
        
        # 3. Build the Cost Function Matrices for Least Squares
        # We want to minimize: (X_future - X_ref)^2 + (U_future)^2
        # This is a standard "Least Squares" problem: Ax = b
        # Here, we solve for U_opt
        
        # Construct large Q and R diagonal matrices
        Q_bar = np.kron(np.eye(self.N), self.Q)
        R_bar = np.kron(np.eye(self.N), self.R)
        
        # The Magic Math (Analytic Solution for Unconstrained MPC)
        # H = S.T * Q * S + R
        H = S.T @ Q_bar @ S + R_bar
        
        # f = S.T * Q * (T * x_current - X_ref)
        x_curr_flat = current_state.flatten()
        prediction_error = (T @ x_curr_flat) - X_ref
        f = S.T @ Q_bar @ prediction_error
        
        # 4. Solve: H * U = -f
        # This gives us the optimal sequence of inputs [u_0, u_1, ... u_N]
        U_opt = np.linalg.solve(H, -f)
        
        # We only take the first step (Receding Horizon)
        return U_opt[0]

    def _build_batch_matrices(self):
        """
        Helper to stack A and B matrices.
        Ask AI: "Write a python function to construct the condensed MPC formulation matrices."
        """
        state_dim = self.A.shape[0]
        control_dim = self.B.shape[1]
        
        # T matrix (Influence of initial state)
        T = np.zeros((state_dim * self.N, state_dim))
        # S matrix (Influence of future inputs)
        S = np.zeros((state_dim * self.N, control_dim * self.N))
        
        A_pow = np.eye(state_dim)
        
        for k in range(self.N):
            row = k * state_dim
            
            # Fill T
            A_pow = A_pow @ self.A
            T[row:row+state_dim, :] = A_pow
            
            # Fill S
            for j in range(k + 1):
                col = j * control_dim
                # Power of A determines how "old" inputs affect current state
                steps_ago = k - j
                mat = np.linalg.matrix_power(self.A, steps_ago) @ self.B
                S[row:row+state_dim, col:col+control_dim] = mat
                
        return T, S 
