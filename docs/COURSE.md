# The "Squid" Comprehensive Curriculum
**Platform:** Pavo20 / Pi Zero 2 W / ROS2 / RL
**Philosophy:** Accessible Entry $\to$ Rigorous Engineering $\to$ Novel Research.

---

## **PHASE I: THE MAKER (Hardware & Foundations)**
*Goal: Build the machine and understand the low-level "Metal."*

### **Module 0: The Physical Layer**
*   **0.1:** Anatomy & Power Systems (BECs, Voltage Sags, Magic Smoke).
*   **0.2:** Soldering & Assembly (Thermal management for the Pi).
*   **0.3:** Linux Systems Engineering (Headless setup, Real-Time Kernels, UART configuration).
*   **0.4:** The "Smoke Test" & Safety Protocols.

### **Module 1: The Bare Metal API**
*   **1.1:** The MSP Protocol: Bit-banging Serial commands to the Flight Controller.
*   **1.2:** I2C Drivers: Writing raw Python drivers for the ToF and Flow sensors.
*   **1.3:** The "Game Loop": Writing non-blocking code without an OS (Understanding Timing).
*   **Capstone:** A Python script that spins motors proportional to distance sensor readings.

---

## **PHASE II: THE DEVELOPER (Telemetry & Visualization)**
*Goal: Use Web skills to build tools that visualize the invisible.*

### **Module 2: The Telemetry Stack**
*   **2.1:** Networking Physics: UDP vs TCP, Latency, and Jitter analysis.
*   **2.2:** The Backend: Writing a high-performance WebSocket server on the Pi.
*   **2.3:** The Frontend: Building a React/HTML5 Dashboard for real-time plotting (50Hz).
*   **Capstone:** Flying the drone via Keyboard while watching a live altitude graph in Chrome.

### **Module 3: Video & FPV**
*   **3.1:** The Video Pipeline: MJPEG vs H.264, Hardware Encoding (MMAL).
*   **3.2:** Augmented Reality: Overlaying sensor data (HUD) onto the video feed.

---

## **PHASE III: THE ENGINEER (Control & Math)**
*Goal: Stop hacking and start Engineering. The transition to Rigor.*

### **Module 4: Signal Processing (The Math of Reality)**
*   **4.1:** Vibration Analysis: FFTs, Aliasing, and Sampling Theorems.
*   **4.2:** Filter Design: Implementing Digital Low-Pass and Notch Filters.
*   **4.3:** Camera Calibration: Intrinsic Matrices and Distortion correction.
*   **Theory Requirement:** *Paper 0.1 (The Mathematician)* - Complex Numbers & Frequency Domain.

### **Module 5: Control Theory**
*   **5.1:** The Feedback Loop: P, I, D explained mathematically.
*   **5.2:** Implementation: Writing a modular PID class with Anti-Windup.
*   **5.3:** System Identification: Measuring the drone's physical response curve.
*   **Capstone:** Autonomous Hover stable to within ±5cm.

---

## **PHASE IV: THE ARCHITECT (Professional Middleware)**
*Goal: Porting everything to Industry Standards (ROS2).*

### **Module 6: The ROS2 Upgrade**
*   **6.1:** Architecture: Nodes, Topics, Services, DDS.
*   **6.2:** The Port: Rewriting the Python "Backend" into ROS2 Nodes.
*   **6.3:** Data Ops: MCAP Logging, Bag Recording, and Replay (Foxglove Studio).
*   **6.4:** Safety Systems: Watchdogs and Failsafe State Machines.

### **Module 7: State Estimation (The Hard Math)**
*   **7.1:** Probability: Bayes Rule and Gaussian Distributions.
*   **7.2:** The Kalman Filter: Deriving the EKF Jacobians for 3D flight.
*   **7.3:** Sensor Fusion: Fusing Optical Flow (Velocity) + IMU (Accel) + ToF (Position).
*   **Capstone:** "The Push Test" - Recovering from violent disturbances using fused data.

---

## **PHASE V: THE RESEARCHER (Advanced Autonomy)**
*Goal: Solving open problems in Robotics.*

### **Module 8: Perception & Mapping**
*   **8.1:** 3D Data: Point Clouds, Voxels, and Octrees.
*   **8.2:** Mapping: Building an Occupancy Grid of the room using the 8x8 ToF.
*   **8.3:** Novelty: Tactile Mapping (Collision-based) & Optical Stealth (Luminance mapping).

### **Module 9: Trajectory Optimization**
*   **9.1:** Pathfinding: A* and JPS (Jump Point Search).
*   **9.2:** Smoothing: Minimum Snap Trajectories (Polynomial Splines).
*   **9.3:** Physics: Exploiting Ground Effect and Ceiling Effect (Perching).

---

## **PHASE VI: THE SPECIALIST (AI & Combat)**
*Goal: Sim-to-Real Transfer and Tactical behaviors.*

### **Module 10: Reinforcement Learning**
*   **10.1:** Simulation: Building the Gym-PyBullet Custom Environment.
*   **10.2:** Training: PPO, Reward Engineering, and Domain Randomization.
*   **10.3:** Deployment: Running the ONNX/PyTorch model on the Pi Zero.

### **Module 11: Tactical Engagement**
*   **11.1:** Guidance Laws: Proportional Navigation (Missile math).
*   **11.2:** Visual Servoing: High-speed tracking/locking.
*   **11.3:** Maneuvers: Zero-G Dives and Ballistic Interception.
*   **Final Thesis:** "The Dark Room" - Autonomous Infiltration & Interception Mission.

---

## **Graduation Requirements**
1.  **Code:** A Git repo with a clean ROS2 workspace and a web-based telemetry stack.
2.  **Math:** Jupyter Notebooks proving the EKF and Spline derivations.
3.  **Data:** Flight logs validating every Capstone.
4.  **Hardware:** A battle-tested drone that has survived the full curriculum.
