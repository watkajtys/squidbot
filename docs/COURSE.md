# The Unified "Squid" Curriculum
**From Zero to Autonomous Hunter**
**Platform:** Pavo20 / Pi Zero 2 W / ROS2 / RL

---

## **PART I: THE UNDERGRADUATE (Foundations)**
*Goal: Build the drone, learn the Linux environment, and achieve stable hover using simple Python scripts. No complex middleware yet.*

### **Module 0: The Build (Hardware)**
*   **0.1:** Anatomy & Assembly (Soldering, BEC wiring).
*   **0.2:** The "Smoke Test" & Safety.
*   **0.3:** Linux Setup (Headless Pi, SSH, Systemd).
*   **0.4:** The Hardware Build Guide (Temperature management, mounting).

### **Module 1: The Internal API (Python Scripting)**
*   **1.1:** Talking to the Flight Controller (UART/MSP).
*   **1.2:** Reading the Senses (I2C/ToF/Flow).
*   **1.3:** The "Reflex" Agent (Simple `if/else` obstacle avoidance).
*   **Capstone:** A script that spins motors when you cover the sensor.

### **Module 2: The Control Loop (PID)**
*   **2.1:** Theory: The Feedback Loop (P, I, D).
*   **2.2:** Implementation: Writing a `pid.py` class from scratch.
*   **2.3:** Tuning: Adjusting gains for a stable 1-meter hover.
*   **Capstone:** The "Floor is Lava" autonomous hover.

---

## **PART II: THE ENGINEER (Systems & Math)**
*Goal: Transition to professional tools (ROS2), derive the math, and build a robust state estimator.*

### **Module 3: The Architecture (ROS2)**
*   **3.1:** Introduction to Nodes, Topics, and Messages.
*   **3.2:** Porting the Python scripts to ROS2 Nodes.
*   **3.3:** Data Ops: Logging flight data with MCAP/Foxglove.
*   **3.4:** Latency & Real-Time Constraints (The Watchdog).

### **Module 4: Signal Processing & Math (The Deep Dive)**
*   **4.1:** Vibration Analysis (FFT & Notch Filtering).
*   **4.2:** Camera Calibration (Intrinsic/Extrinsic Matrices).
*   **4.3:** Coordinate Transformations (Quaternions & Frames).
*   **Reference:** *Paper 0.1: The Mathematician.*

### **Module 5: State Estimation (EKF)**
*   **5.1:** Theory: Bayes Filter & Kalman Filter.
*   **5.2:** Derivation: The Extended Kalman Filter (Jacobians).
*   **5.3:** Implementation: Fusing Optical Flow + IMU + ToF.
*   **Capstone:** "The Push Test" (Recovering from disturbances).

---

## **PART III: THE RESEARCHER (Advanced Autonomy)**
*Goal: Advanced Perception, Trajectory Optimization, and Reinforcement Learning.*

### **Module 6: Perception & Mapping**
*   **6.1:** 3D Point Clouds (From 8x8 ToF).
*   **6.2:** Voxel Grids & Occupancy Maps.
*   **6.3:** Novelty: Tactile Mapping (Bumper Car) & Optical Stealth.
*   **Capstone:** "The Ghost Map" (Digital Twin generation).

### **Module 7: Planning & Dynamics**
*   **7.1:** Pathfinding (A* / JPS).
*   **7.2:** Trajectory Optimization (Minimum Snap Splines).
*   **7.3:** Physics Exploitation: Ceiling Perching & Draft Hunting.

### **Module 8: Reinforcement Learning (Sim-to-Real)**
*   **8.1:** Simulation: Building the Gym-PyBullet environment.
*   **8.2:** Training: PPO & Domain Randomization.
*   **8.3:** Deployment: Running the Neural Network on the Pi.

---

## **PART IV: THE SPECIALIST (Tactical Engagement)**
*Goal: Dynamic, adversarial scenarios.*

### **Module 9: Aerial Combat**
*   **9.1:** Guidance Laws (Pro-Nav).
*   **9.2:** Visual Servoing (Lock-on).
*   **9.3:** Zero-G Maneuvering (Ballistic Interception).
*   **Capstone:** "The Dogfight" (Intercepting a moving target).

---

## **Graduation Requirements**
1.  **Hardware:** A working Pavo20 with custom 3D mount.
2.  **Code:** A Git repo with `scripts/` (Part I) and `ros2_ws/` (Part II+).
3.  **Theory:** Notebooks deriving the EKF and Splines.
4.  **Data:** Logs proving successful autonomous interception.
