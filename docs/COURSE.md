# The "Squid" Integrated Curriculum (Verified)
**Standard:** Matches ETH Zurich / UPenn MEAM 620
**Flow:** Hardware $\to$ Software $\to$ Control $\to$ Estimation $\to$ Intelligence.

---

## **PHASE I: THE MECHANIC (Hardware & Instincts)**



### **Module 0: The Foundation**

*   **0.A:** [Hardware Foundations](Hardware_Foundations.md) - Electronics, Soldering, and Tools.

*   **0.B:** [Theory Handbook](theory/Theory_0_Concepts.md) - Vibration, Camera Models, and Real-Time Physics.

*   **0.C:** [Pre-Flight Labs](Module_0_Labs.md) - Bench testing, Linux, and Wi-Fi mapping.

*   **0.D:** [The Build Guide](Module_0_The_Build.md) - Assembly and Headless Linux Setup.

    *   **Check:** The Smoke Test (Connectivity Verification).



### **Module 1: The Bare Metal API**

*   **1.0:** [The Heartbeat](Module_1_The_Bare_Metal_API.md#10-lab-the-heartbeat-your-first-success) - Blinking the onboard LED.

*   **1.1:** MSP Protocol: Byte-level motor control.

    *   **Sub-Lab:** **The UART Sanity Check.** ASK the FC for its version before spinning motors.

    *   **Sub-Lab:** **The MSP Hex Dump.** View the raw bytes.

    *   **The Checksum Challenge:** Manually XOR the bytes to verify protocol integrity.

*   **1.2:** I2C Drivers: Raw sensor access.

    *   **Sub-Lab:** **The I2C Scanner.** Prove you can see the wires before writing the driver.

*   **1.3:** The Game Loop: Timing & Jitter.

*   **Check:** The Reflex (Sensor-based throttle).



---



## **PHASE II: THE TEST PILOT (Observability)**



### **Module 2: The Telemetry Stack**

*   **2.1:** Networking: UDP Sockets & Latency.

    *   **Sub-Lab:** **The Packet Loss Simulation.** Dropping packets to test robustness.

    *   **Sub-Lab:** **The Jitter Test.** Adding random delay to see the "Stutter" effect.

*   **2.2:** The Dashboard: React/HTML5 Real-time plotting.

*   **2.3:** The Logger: High-frequency CSV logging.

    *   **Sub-Lab:** **CSV to Plot.** Post-flight autopsy of loop stability.

*   **Check:** The Flight Recorder (Data analysis).



### **Module 3: FPV & HUD**

*   **3.1:** Video Pipeline: Hardware Encoding.

*   **3.2:** Augmented Reality: Overlaying Data.

    *   **Sub-Lab:** **The Virtual Horizon.** Drawing a line that stays level with the earth.

*   **Check:** Instrument Flight.



---



## **PHASE III: THE ENGINEER (Control & Math)**



### **Module 4: Signal Processing & Geometry**

*   **4.1:** Coordinate Frames: [Theory 1: Coordinate Systems](theory/Theory_1_Coordinate_Systems.md).

*   **4.2:** Vibration Analysis: FFTs & Notch Filters.

    *   **Sub-Lab:** **Phase Lag Visualization.** Measuring the time-cost of your filters.

*   **4.3:** System ID: [Measuring Thrust & Inertia](../src/labs/lab_4_sysid.py).

*   **Check:** The Flatline (Clean Data).



### **Module 5: Control Theory**

*   **Prerequisite:** [Theory 4: PID to LQR](theory/Theory_4_PID_to_LQR.md).

*   **5.1:** The Feedback Loop: PID Derivation.

*   **5.2:** Implementation: [The Anti-Windup Guard](Module_5_Control_Theory.md#521-sub-lab-the-anti-windup-guard).

*   **5.3:** Tuning: Empirical Ziegler-Nichols.

    *   **Sub-Lab:** **The Oscillation Hunt.** Scientifically finding the Ultimate Gain ($K_u$).

*   **Check:** The Statue (Precision Hover).



---



## **PHASE IV: THE ARCHITECT (Scale & Standards)**



### **Module 6: The ROS 2 Migration**

*   **6.4:** Safety: [Lifecycle Nodes](Module_6_ROS2_Migration.md#64-safety-architecture-lifecycle-nodes).

*   **Check:** The Replica (Port validation).



### **Module 7: State Estimation (The Truth)**

*   **Prerequisite:** [Theory 2: The EKF](theory/Theory_2_The_EKF.md).

*   **7.3:** Sensor Fusion.

    *   **Sub-Lab:** **The Outlier Rejection Test.** Tricking the robot with a piece of paper to see if the EKF ignores the "Liar."

*   **7.5:** Calibration: [The Compass](Module_7_State_Estimation.md#75-calibration-the-compass).

*   **Check:** The Push Test.



---



## **PHASE V: THE RESEARCHER (Advanced Autonomy)**



### **Module 8: Perception & Mapping**

*   **Prerequisite:** [Theory 5: SLAM & Factor Graphs](theory/Theory_5_SLAM_and_Factor_Graphs.md).

*   **8.1:** Point Clouds.

    *   **Sub-Lab:** **The Mirror Mystery.** Seeing the "Ghost Room" in reflections.

*   **Check:** The Ghost Map.



### **Module 9: Trajectory Optimization**

*   **Prerequisite:** [Theory 3: Splines & Optimization](theory/Theory_3_Splines_and_Optimization.md).

*   **9.4:** Advanced: [Geometric Control on SE(3)](Module_9_Trajectory_Optimization.md#94-advanced-tracking-geometric-control-on-se3).

*   **9.5:** Search: [Implementing A-Star](../src/labs/lab_9_planning.py).

*   **Check:** The Speed Run.



---



## **PHASE VI: THE SPECIALIST (Tactical Engagement)**



### **Module 10: Reinforcement Learning**

*   **10.1:** Training.

    *   **Sub-Lab:** **Reward Hacking.** Watching an AI learn to "cheat" a bad reward function.

    *   **Sub-Lab:** **The Digital Twin Audit.** Comparing a flight log to the Sim to measure "Reality Gap."

*   **Check:** The Uncrashable Drone.



### **Module 11: Aerial Combat**

*   **11.4:** Advanced: [The QR Courier](Module_11_Aerial_Combat_and_Guidance.md#114-sub-lab-the-qr-courier).

*   **Check:** The Dark Room Scenario.



### **Module 12: Outdoor Autonomy**

*   **Prerequisite:** [Theory 7: Behavior Trees](theory/Theory_7_Behavior_Trees.md).

*   **12.1:** GPS.

    *   **Sub-Lab:** **The GPS Treasure Hunt.** Walking the math to verify NED transforms.

*   **Check:** The Mile Run.



---



## **PHASE VII: THE FRONTIER (PhD Level)**



### **Module 13: Visual Inertial Odometry (VIO)**

*   **Prerequisite:** [Theory 6: VIO & Preintegration](theory/Theory_6_VIO_and_Preintegration.md).

*   **Theory Bonus:** [The Initialization Problem](theory/Theory_6_VIO_and_Preintegration.md#5-the-initialization-problem).

*   **Check:** The Loop Closure.



### **Module 14: Swarm Theory**

*   **14.2:** Lab: [The Consensus Drill](Module_14_Swarm_Theory.md#142-sub-lab-the-consensus-drill).

*   **Check:** The Synchronized Dance.



### **Module 15: Deep Perception**

*   **15.1:** Lab: [Siamese Networks for Loop Closure].

*   **Check:** The Memory (Global Localization).
