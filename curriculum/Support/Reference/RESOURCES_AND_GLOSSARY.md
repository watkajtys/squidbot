# Resources & Glossary
**The "Squid" Drone Project Reference Library**

This document supports the Master Curriculum. Refer to this when you encounter new terms or tools.

---

## 📚 The Self-Taught Curriculum (Recommended Reading)

### **1. The Prerequisites (Week 0)**
*   **Linux CLI:** [Linux Journey](https://linuxjourney.com/) (Focus on: Command Line, Permissions, Processes).
*   **Python:** [Real Python](https://realpython.com/) (Focus on: Classes, Threading, Sockets, NumPy).
*   **Git:** [Pro Git Book](https://git-scm.com/book/en/v2) (Focus on: Basic branching and committing).

### **2. Middleware (ROS 2)**
*   **Official Docs:** [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/).
*   **The "Bible":** [Articulated Robotics (YouTube)](https://www.youtube.com/@ArticulatedRobotics). *Watch his "Zero to Hero" series.*
*   **Visualization:** [Foxglove Studio Docs](https://docs.foxglove.dev/docs/introduction).

### **3. The Math (State Estimation & Control)**
*   **Linear Algebra:** [3Blue1Brown - Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab). *Crucial for Kalman Filters.*
*   **Kalman Filters:** [Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python). *Free interactive textbook.*
*   **Control Theory:** [Brian Douglas - Control Systems](https://www.youtube.com/user/ControlLectures). *The best explanation of PID/Bode plots.*

### **4. Artificial Intelligence (RL)**
*   **Theory:** [OpenAI Spinning Up](https://spinningup.openai.com/en/latest/). *The best intro to PPO and Policy Gradients.*
*   **Implementation:** [Stable Baselines3 Docs](https://stable-baselines3.readthedocs.io/).
*   **Simulation:** [Gym-PyBullet-Drones Repo](https://github.com/utiasDSL/gym-pybullet-drones).

### **5. The Frontier (2025+ Research)**
*   **Liquid Neural Networks:** [MIT CSAIL - Liquid Neural Networks](https://news.mit.edu/2021/liquid-machine-learning-0128). *The future of adaptive edge AI.*
*   **3D Gaussian Splatting:** [Luma AI / 3DGS Paper](https://repo-lpni.github.io/3dgs/). *Differentiable 3D representation.*
*   **Reachability:** [Berkeley Hybrid Systems Lab](https://hybrid.eecs.berkeley.edu/). *Hamilton-Jacobi Reachability for safety.*

### **6. Local Project Guides**
*   [The Pi Zero Survival Guide](../Setup/PI_ZERO_SURVIVAL_GUIDE.md) - Power and thermal management.
*   [Hardware Component Deep Dive](../Hardware/HARDWARE_DEEP_DIVE.md) - Physics and math of the sensors.
*   [Hardware Foundations](../Hardware/Hardware_Foundations.md) - Electronics and soldering 101.

---

## 📖 Glossary of Terms

### **Hardware & Electronics**
*   **BEC (Battery Eliminator Circuit):** A voltage regulator. Takes 12V from the battery and gives clean 5V to the Pi. Essential for preventing brownouts.
*   **DShot:** A digital protocol for controlling motors. Much faster and more accurate than the old Analog PWM.
*   **UART (Universal Asynchronous Receiver-Transmitter):** Serial communication (TX/RX). Used between the Flight Controller and the Pi.
*   **I2C (Inter-Integrated Circuit):** A bus for connecting sensors (ToF, Mag). Uses two wires (SDA, SCL).

### **Robotics & Control**
*   **Holonomic vs. Non-Holonomic:**
    *   *Holonomic:* Can move in any direction (like a drone).
    *   *Non-Holonomic:* Constrained movement (like a car; it cannot slide sideways).
*   **PID (Proportional-Integral-Derivative):** The standard control loop.
*   **MPC (Model Predictive Control):** A 'Chess Grandmaster' controller that looks $N$ steps into the future to optimize the current command while respecting constraints.
*   **CBF (Control Barrier Function):** A mathematical 'Guardian' that filters control inputs to ensure the drone never leaves a 'Safe Set' (hitting a wall).
*   **HJI (Hamilton-Jacobi-Isaacs):** A differential equation used in reachability analysis to calculate 'Capture Sets'.
*   **EKF (Extended Kalman Filter):** Fuses noisy sensor data into a single, accurate estimate of position.

### **Perception & Mapping**
*   **SLAM (Simultaneous Localization and Mapping):** Building a map while figuring out where you are inside it.
*   **3DGS (3D Gaussian Splatting):** Representing the world as millions of tiny, differentiable 3D ellipsoids.
*   **Voxel:** A "3D Pixel." We use Voxel Grids to represent the room as a block world.
*   **Point Cloud:** A raw collection of X,Y,Z points returned by the Lidar/ToF sensor.

### **AI & Reinforcement Learning**
*   **LNN (Liquid Neural Network):** Neural networks based on continuous-time differential equations that can 'adapt' time-constants in real-time.
*   **Sim-to-Real Gap:** When an AI works in simulation but fails in real life.
*   **PPO (Proximal Policy Optimization):** The industry standard RL algorithm for robotics.

---

## 🛠 Required Software Stack

1.  **VS Code:** Your Code Editor.
2.  **Docker:** For running ROS 2 containers without breaking your laptop's OS.
3.  **QGroundControl / Betaflight Configurator:** For low-level drone setup.
4.  **OpenSCAD:** For editing the 3D printed mount.
5.  **Foxglove Studio:** The industry-standard web-based visualizer. Used for viewing 3D Coordinate Frames (TFs), Point Clouds, and Telemetry in real-time.--- [Return to Course Map](../../../COURSE_MAP.md)