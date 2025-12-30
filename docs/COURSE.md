# The "Squid" Autonomous Robotics Master Curriculum (Final)
**Platform:** Pavo20 / Pi Zero 2 W / ROS2 Middleware / Offboard Compute
**Standard:** Graduate Level Research (Stanford CS287 / MIT 16.413 Equivalent)

---

## **Architecture: The Hybrid Brain**
*   **The Body (Drone):** Runs a lightweight **ROS2 Node** on the Pi Zero 2 W. Its only job is to stream raw sensor data and execute motor commands.
*   **The Mind (Laptop):** Runs the heavy **ROS2 Stack**. This is where the EKF, Mapping, Path Planning, and RL Inference happen.
*   **The Visualization:** Use **Foxglove Studio** or **RViz2** on the laptop to see the drone's "thoughts" (point clouds, vectors, and camera feed) in real-time.

---

## **SEMESTER 1: Foundations & Systems Engineering**

### **Module 0: Signal, Noise & Calibration**
*   **Vibration Analysis:** Use FFT to identify motor resonance. Implement **Bi-directional DShot** (if supported) or software Notch Filters.
*   **Sensor Calibration:** Camera Intrinsic/Extrinsic calibration. IMU thermal calibration (Does your gyro drift as the Pi gets hot?).
*   **FMEA (Failure Analysis):** Designing a "Watchdog" that detects motor desync or sensor "freezes" and initiates an emergency "controlled tumble."

### **Module 1: The ROS2 Nervous System**
*   **Middleware:** Setting up **ROS2 Humble/Iron**. Defining custom `.msg` types for your ToF array.
*   **Data Infrastructure:** Setting up **MCAP (Foxglove)** logging. You must record every flight. If it isn't logged, it didn't happen.
*   **Latency Benchmarking:** Characterizing the Wi-Fi "Jitter." Modeling this delay in your simulation (Time-Delay Compensation).
*   **Theory:** Publish/Subscribe architecture vs. Request/Response (Services).

### **Module 2: State Estimation (The Math Core)**
*   **Theory:** Hand-derivation of the **Extended Kalman Filter (EKF)**.
*   **Implementation:** Fusing Optical Flow + 1D Lidar + IMU.
*   **Novelty (Drifting):** Implementing **Decoupled Yaw Control** (Non-holonomic constraints). Proving you can fly a circle while the camera stays fixed on a point.

---

## **SEMESTER 2: Perception, Planning & Physics Hackery**

### **Module 3: Mapping & Spatial Intelligence**
*   **3D SLAM:** Using the 8x8 ToF Matrix to build a **Voxel Map**.
*   **Novelty (Tactile):** "Bumper Car" mapping. Using physical impacts to find glass walls.
*   **Novelty (Stealth):** **Luminance Mapping**. Integrating light-intensity into the navigation cost-function (Phototaxis).

### **Module 4: Classical Path Planning**
*   **Search:** Implementing **A*** or **JPS (Jump Point Search)** on your 3D Voxel map.
*   **Optimization:** Converting jagged paths into **Minimum Snap Trajectories** (Polynomial Splines) for surgical, smooth flight.
*   **Theory:** Constrained Optimization (Lagrange Multipliers).

### **Module 5: Physics Exploitation (SpecOps)**
*   **Ceiling Spider:** Aerodynamic Suction Perching.
*   **Toss-to-Launch:** 6-DOF recovery from arbitrary orientation.
*   **Acoustic Stealth:** RPM-syncing to minimize "Beat Frequencies" and motor whine.
*   **Draft Hunting:** Using the "I" term of your PID to detect air currents from windows/vents.

---

## **SEMESTER 3: Advanced AI & Tactical Engagement**

### **Module 6: Sim-to-Real Reinforcement Learning**
*   **Environment:** **Gym-PyBullet-Drones**. Custom URDF with your Pavo20's specific physics.
*   **Algorithm:** **PPO (Proximal Policy Optimization)**.
*   **Technique:** **Domain Randomization**. Randomizing mass, thrust, and latency in sim to ensure the brain works in the messy real world.

### **Module 7: The Interceptor (Aerial Combat)**
*   **Guidance:** **Proportional Navigation (Pro-Nav)**. Missile-grade interception laws.
*   **Dogfighting (Self-Play):** Training an RL agent to "win" a game of tag against an earlier version of itself.
*   **Zero-G Dive:** Silent ballistic interception (motors off) from high-altitude perches.

---

## **POST-GRADUATE: The Horizon**
*Where to go after you master the single agent.*

### **Module 8: Future Frontiers**
*   **Swarm Intelligence:** Distributed SLAM (Map merging) with multiple drones.
*   **Hardware V2:** Moving from Pi Zero 2 W to a custom PCB with **STM32 + NPU** (Neural Processing Unit) for onboard AI.
*   **Visual Odometry:** Replacing Optical Flow with Visual SLAM (ORB-SLAM3) for GPS-denied outdoor navigation.

---

## **FINAL THESIS: "The Dark Room Scenario"**
**A single mission integrating all modules:**
1.  **Deployment:** Hand-toss into a pitch-black room.
2.  **Infiltration:** Tactile mapping of the entrance + Optical Stealth navigation.
3.  **Surveillance:** Finding the target (Roomba) using the Arducam.
4.  **Assault:** A **Minimum Snap** high-speed approach followed by a **Zero-G Dive** for a silent "strike."
5.  **Exfiltration:** Navigation back to the start and **Suction Perch** on the ceiling.

---

## **Graduation Requirements**
1.  **Theory:** A GitHub folder `/theory` with Jupyter Notebooks proving the math for EKF, Splines, and Pro-Nav.
2.  **Code:** A clean, documented **ROS2 Workspace**.
3.  **Data:** A library of `.mcap` flight logs proving every experiment.
4.  **Video:** A "Highlights Reel" showing the success of each Module's Capstone Experiment.
