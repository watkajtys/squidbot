# The "Squid" Integrated Curriculum (Verified)
**Standard:** Matches ETH Zurich / UPenn MEAM 620
**Flow:** Hardware $\to$ Software $\to$ Control $\to$ Estimation $\to$ Intelligence.

---

## **PHASE I: THE MECHANIC (Hardware & Instincts)**

### **Module 0: The Foundation**
*   **0.A:** [Theory Handbook](Theory_0_Concepts.md) - Vibration, Camera Models, and Real-Time Physics.
*   **0.B:** [The Build Guide](Module_0_The_Build.md) - Soldering, Assembly, and Headless Linux Setup.
    *   **0.C:** [Legal & Safety](Module_0_The_Build.md) - FAA TRUST, Remote ID, and Insurance.
    *   **Check:** The Smoke Test (Connectivity Verification).
### **Module 1: The Bare Metal API**
*   **1.1:** MSP Protocol: Byte-level motor control.
    *   **Sub-Lab:** **The Logic Analyzer.** Use a $10 USB analyzer to capture the DSHOT/PWM packets. Prove that "50% Throttle" in code equals 50% duty cycle on the wire.
*   **1.2:** I2C Drivers: Raw sensor access (Solving the Address Conflict).
*   **1.3:** The Game Loop: Timing & Jitter.
    *   **Sub-Lab (Lect 0.1):** **The Jitter Plot.** Log the time difference ($\Delta t$) between 1,000 loops. Plot a Histogram. Calculate the Standard Deviation. Prove your loop is "Real-Time Enough."
*   **Check:** The Reflex (Sensor-based throttle).

---

## **PHASE II: THE TEST PILOT (Observability)**

### **Module 2: The Telemetry Stack**
*   **2.1:** Networking: UDP Sockets & Latency.
*   **2.2:** The Dashboard: React/HTML5 Real-time plotting.
*   **2.3:** The Logger: High-frequency CSV logging.
    *   **Sub-Lab (Lect 1.2):** **The Corrupt Packet.** Intentionally inject bit-errors into your UDP stream. Implement a CRC check to detect and reject them.
*   **Check:** The Flight Recorder (Data analysis).

### **Module 3: FPV & HUD**
*   **3.1:** Video Pipeline: Hardware Encoding.
*   **3.2:** Augmented Reality: Overlaying Data (Multi-ToF telemetry).
    *   **Sub-Lab (Lect 3.4):** **The Photon-to-Motion Loop.** Measure the total system latency ($T_{pipeline}$) using a high-speed camera and an LED.
*   **Check:** Instrument Flight.

---

## **PHASE III: THE ENGINEER (Control & Math)**

### **Module 4: Signal Processing & Geometry**
*   **4.1:** Coordinate Frames: Rotations & Quaternions.
    *   **Sub-Lab (Lect 4.1):** **Gimbal Lock Simulation.** Write a script that uses Euler Angles to rotate a vector. Find the exact pitch angle (90 deg) where you lose a degree of freedom. Fix it with Quaternions.
*   **4.2:** Vibration Analysis: FFTs & Notch Filters.
    *   **Sub-Lab (Lect 2.3):** **The Allan Variance.** Record 1 hour of *stationary* gyro data. Plot the Log-Log graph. Identify the "Bias Instability" point (where the curve bottoms out).
    *   **Sub-Lab:** **The Nyquist Shadow.** Downsample 1000Hz vibration data to 50Hz without filtering. Observe the high-frequency motor noise appearing as a fake low-frequency "wobble."
*   **4.3:** Camera Calibration: Intrinsic Camera Matrices.
*   **4.4:** Extrinsic Calibration: Measuring the Camera-IMU offset ($T_{bc}$).
*   **4.5:** Magnetometer Calibration: Hard/Soft Iron correction.
*   **Check:** The Flatline (Clean Data).

### **Module 5: Control Theory**
*   **5.1:** The Feedback Loop: PID Derivation.
    *   **Sub-Lab:** **The Drunk Pilot.** Intentionally inject a 100ms delay buffer into your sensor stream. Observe the immediate loss of stability (Phase Margin collapse).
*   **5.2:** Implementation: Anti-Windup & Derivative Kick.
*   **5.3:** Tuning: Empirical Ziegler-Nichols.
        *   **Sub-Lab (Lect 5.2):** **System Identification.** Send a "Chirp Signal" (Sine wave increasing in frequency) to the motors while the drone is tied down. Record the IMU response. Use MATLAB/Python to fit a Transfer Function.
        *   **Sub-Lab (Lect 5.4):** **The Bifilar Pendulum.** Measure the exact Moment of Inertia ($I_{xx}, I_{yy}, I_{zz}$) by hanging the drone from two strings and timing the oscillation.
        *   **Sub-Lab:** **The Static Thrust Stand.** Measure Grams/Amp efficiency and derive the Thrust Coefficient ($k_t$).
    *   **Check:** The Statue (Precision Hover).
---

## **PHASE IV: THE ARCHITECT (Scale & Standards)**

### **Module 6: The ROS2 Migration**
*   **6.1:** Architecture: Nodes, Topics, DDS.
*   **6.2:** The Port: Rewriting Python to ROS2.
*   **6.3:** Data Ops: MCAP Logging.
    *   **Sub-Lab (Lect 6.4):** **The Watchdog.** Implement a Fail-Safe State Machine that detects "Loss of Comms" or "Vision Failure" and triggers an emergency landing.
*   **Check:** The Replica (Port validation).

### **Module 7: State Estimation (The Truth)**
*   **7.1:** Time Synchronization: Interpolation.
*   **7.2:** The EKF: Jacobian Derivation.
*   **7.3:** Sensor Fusion (Lidar + Optical Flow + IMU).
    *   **Sub-Lab (Lect 7.2):** **The Covariance Tune.** Manually edit the $Q$ (Process Noise) and $R$ (Measurement Noise) matrices. Observe how the EKF "Lag" changes when you trust the model vs. the sensors.
    *   **Sub-Lab (Lect 7.4):** **The Battery Estimator.** Implement a Coulomb Counting + Voltage Sag filter to estimate State-of-Charge (SoC) under load.
*   **Check:** The Push Test.

---

## **PHASE V: THE RESEARCHER (Advanced Autonomy)**

### **Module 8: Perception & Mapping**
*   **8.1:** Point Clouds & Voxel Grids.
    *   **Sub-Lab:** **The Mirror World.** Point the Lidar at a mirror. Observe the "Ghost Room" generated behind it. Use this to understand Multi-path interference.
*   **8.2:** Occupancy Mapping (Octomap).
    *   **Sub-Lab (Lect 9.2):** **The Ray Cast.** Implement the Bresenham Line Algorithm manually to clear "Free Space" pixels between the drone and the wall.
*   **8.3:** The Digital Twin Pipeline (Export to Sim).
*   **Check:** The Ghost Map.

### **Module 9: Trajectory Optimization**
*   **9.1:** Pathfinding: A* / JPS.
*   **9.2:** Smoothing: Minimum Snap Splines.
        *   **Sub-Lab (Lect 10.3):** **The Snap Solver.** Use `scipy.optimize` (QP Solver) to fit a polynomial through 3 points while minimizing the 4th derivative. Compare it to a standard Cubic Spline.
    *   **9.3:** Model Predictive Control (MPC): Introduction to Receding Horizon Control.
    *   **Check:** The Speed Run.
---

## **PHASE VI: THE SPECIALIST (Tactical Engagement)**

### **Module 10: Reinforcement Learning**
*   **10.1:** Training PPO in Gym.
*   **10.2:** Sim-to-Real: Domain Randomization.
*   **10.3:** Deployment (ONNX).
*   **Check:** The Uncrashable Drone.

### **Module 11: Aerial Combat**
*   **11.1:** Pro-Nav Guidance Laws.
    *   **Sub-Lab (Lect 12.2):** **The Intercept.** Simulate a "Runner" and a "Chaser" in 2D Python. Prove that Pro-Nav intercepts faster than "Pure Pursuit."
*   **11.2:** Visual Servoing.
    *   **Sub-Lab:** **The Red Balloon.** Implement an HSV color tracker to lock onto a balloon. Program the drone to "Ram" it by maximizing the blob area.
*   **Check:** The Dark Room Scenario.

### **Module 12: Outdoor Autonomy**
*   **12.1:** GPS Integration & Coordinate Transforms (LLA to NED).
*   **12.2:** Hybrid Navigation (Indoor/Outdoor Switching).
*   **12.3:** Behavior Trees: Replacing State Machines with `py_trees`.
*   **Check:** The Mile Run.---

## **PHASE VII: THE FRONTIER (PhD Level)**

### **Module 13: Visual Inertial Odometry (VIO)**
*   **13.1:** **The Dataset:** Record a flight with *perfect* time-sync between Camera and IMU.
*   **13.2:** **Feature Tracking:** Implement KLT (Kanade-Lucas-Tomasi) Optical Flow to track feature points across frames.
*   **13.3:** **The VIO Pipeline:** Use a library like `OpenVINS` or `VINS-Mono` (or implement a basic pre-integration solver). Feed it your data.
*   **Sub-Lab (Lect 13.5):** **The Factor Graph.** Use the `GTSAM` library to optimize a simple graph: Node A (Pose 1) -> Constraint (IMU) -> Node B (Pose 2).
*   **Check:** **The Loop Closure.** Fly a circle. Return to start. Does the estimated position return to (0,0,0), or has it drifted?

### **Module 14: Swarm Theory (Preview)**
*   **14.1:** **The Mesh:** Configure ROS2 DDS to talk across multiple Pis (`ROS_DOMAIN_ID`).
*   **14.2:** **Consensus:** Implement a distributed average algorithm so all drones agree on the "Center of Swarm."
*   **Check:** **The Flocking.** One pilot flies Drone A. Drone B and Drone C automatically follow in a triangle formation.