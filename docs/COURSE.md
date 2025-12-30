# The "Squid" Integrated Curriculum (Verified)
**Standard:** Matches ETH Zurich / UPenn MEAM 620
**Flow:** Hardware $\to$ Software $\to$ Control $\to$ Estimation $\to$ Intelligence.

---

## **PHASE I: THE MECHANIC (Hardware & Instincts)**

### **Module 0: The Physical Layer**
*   **0.1:** Anatomy & Power: 12V/5V/3.3V logic.
*   **0.2:** Assembly: Soldering & Thermal Management.
*   **0.3:** Systems: Headless Linux, UART, and Real-Time Kernels.
*   **Check:** The Smoke Test.

### **Module 1: The Bare Metal API**
*   **1.1:** MSP Protocol: Byte-level motor control.
*   **1.2:** I2C Drivers: Raw sensor access. 
    *   **The Conflict:** Solving the I2C address collision for the **2x VL53L1X** lidars using the XSHUT pins.
*   **1.3:** The Game Loop: Timing & Jitter.
*   **Check:** The Reflex (Sensor-based throttle).

---

## **PHASE II: THE TEST PILOT (Observability)**

### **Module 2: The Telemetry Stack**
*   **2.1:** Networking: UDP Sockets & Latency.
*   **2.2:** The Dashboard: React/HTML5 Real-time plotting.
*   **2.3:** The Logger: High-frequency CSV logging.
*   **Check:** The Flight Recorder (Data analysis).

### **Module 3: FPV & HUD**
*   **3.1:** Video Pipeline: Hardware Encoding.
*   **3.2:** Augmented Reality: Overlaying Data (Multi-ToF telemetry).
*   **Check:** Instrument Flight.

---

## **PHASE III: THE ENGINEER (Control & Math)**

### **Module 4: Signal Processing & Geometry**
*   **4.1:** **Coordinate Frames:** Body Frame vs World Frame. Rotation Matrices & Quaternions.
*   **4.2:** Vibration Analysis: FFTs & Notch Filters.
*   **4.3:** Camera Calibration: Intrinsic Camera Matrices.
*   **4.4:** Magnetometer Calibration: "Hard Iron" vs "Soft Iron" distortion correction. (Crucial for the M10Q Compass).
*   **Check:** The Flatline (Clean Data).

### **Module 5: Control Theory**
*   **5.1:** The Feedback Loop: PID Derivation.
*   **5.2:** Implementation: Anti-Windup & Derivative Kick.
*   **5.3:** Tuning: Empirical Ziegler-Nichols method.
*   **Check:** The Statue (Precision Hover).

---

## **PHASE IV: THE ARCHITECT (Scale & Standards)**

### **Module 6: The ROS2 Migration**
*   **6.1:** Architecture: Nodes, Topics, DDS.
*   **6.2:** The Port: Rewriting Python to ROS2.
*   **6.3:** Data Ops: MCAP Logging.
*   **Check:** The Replica (Port validation).

### **Module 7: State Estimation (The Truth)**
*   **7.1:** **Time Synchronization:** Aligning IMU (1khz) and Camera (30Hz) timestamps.
*   **7.2:** The EKF: Jacobian Derivation & Covariance logic.
*   **7.3:** Sensor Fusion:
    *   **Indoor:** Flow + IMU + ToF (Altitude) + ToF (Ceiling/Obstacle).
    *   **Outdoor:** GPS + Barometer + Compass.
*   **Check:** The Push Test.

---

## **PHASE V: THE RESEARCHER (Advanced Autonomy)**

### **Module 8: Perception & Mapping**
*   **8.1:** Point Clouds & Voxel Grids.
*   **8.2:** Occupancy Mapping.
*   **8.3:** **The Digital Twin Pipeline:** Exporting the Voxel Map to a collision mesh (`.urdf`) for the Simulator.
*   **8.4:** Novelty: Tactile Mapping & Optical Stealth.
*   **Check:** The Ghost Map (and successfully loading it into PyBullet).

### **Module 9: Trajectory Optimization**
*   **9.1:** Pathfinding: A* / JPS.
*   **9.2:** Smoothing: Minimum Snap Splines.
*   **9.3:** Physics: Ceiling Perching.
*   **Check:** The Speed Run.

---

## **PHASE VI: THE SPECIALIST (Tactical Engagement)**

### **Module 10: Reinforcement Learning (The Brain)**
*   **10.1:** **Level 1 (Stability):** Training an RL agent to replace the PID controller (Input: IMU, Output: Motor Thrust).
*   **10.2:** **Level 2 (Obstacles):** Training collision avoidance (Input: 8x8 ToF + Flow, Output: Velocity Command).
*   **10.3:** **Level 3 (Navigation):** Training the agent to traverse "The Digital Twin" (from Module 8) at max speed).
*   **10.4:** **Sim-to-Real:** Domain Randomization to survive real-world noise.
*   **Check:** The "Uncrashable" Drone.

### **Module 11: Aerial Combat**
*   **11.1:** Pro-Nav Guidance Laws.
*   **11.2:** Visual Servoing (Lock-on).
*   **11.3:** Zero-G Maneuvers.
*   **Thesis:** "The Dark Room Scenario."

### **Module 12: Outdoor Autonomy (The Traveler)**
*   **12.1:** GPS Integration: Parsing NMEA/UBLOX packets.
*   **12.2:** Global Navigation: Converting Lat/Lon to Local NED (North-East-Down) meters.
*   **12.3:** Hybrid Navigation: Seamless switching between Optical Flow (Indoor) and GPS (Outdoor).
