# The Squid: Autonomous Micro-Drone Project
**"From Soldering Iron to Swarm Intelligence"**

The Squid is a first-principles autonomous drone platform designed to bridge the gap between hobbyist "black box" flight and University-level robotics research.

## 🎓 The Curriculum: Zero to PhD
This project is structured as a full-stack robotics course, taking you from raw electronics to PhD-level Visual Inertial Odometry (VIO).

### **Roadmap (Phase I - VII)**
1.  **Phase I: The Mechanic** (Hardware, Drivers, Linux)
2.  **Phase II: The Test Pilot** (Telemetry, FPV, Augmented Reality)
3.  **Phase III: The Engineer** (Signal Processing, PID Control, Physics)
4.  **Phase IV: The Architect** (ROS 2 Migration, State Estimation/EKF)
5.  **Phase V: The Researcher** (Perception, Mapping, Trajectory Optimization)
6.  **Phase VI: The Specialist** (Reinforcement Learning, Aerial Combat, GPS)
7.  **Phase VII: The Frontier** (VIO, Swarm Theory)

## 📂 Project Structure
*   **`/docs`**: The Curriculum, Modules, and "Deep Dive" Study Guides.
    *   `SYLLABUS.md`: The high-level roadmap and prerequisites.
    *   `LECTURES.md`: The theoretical study guide with academic reading lists.
    *   `COURSE.md`: The integrated step-by-step experiment guide.
    *   `deep_dives/`: PhD-level theoretical support for complex math.
*   **`/hardware`**: CAD files (`.scad`) and wiring schematics.
*   **`/simulation`**: PyBullet physics environment and URDF models.
*   **`/src`**: Python source code for drivers, controllers, and utilities.
*   **`/tools`**: Automation scripts (e.g., `setup_pi.sh`).

## 🚀 Getting Started (Module 0)
1.  **Prerequisites:** Read [Hardware Foundations](docs/Hardware_Foundations.md) and [The Mathematician](docs/deep_dives/Paper_0.1_The_Mathematician.md).
2.  **Environment:** Run `sudo ./tools/setup_pi.sh` on your Raspberry Pi Zero 2 W.
3.  **Labs:** Complete the [Module 0 Bench Labs](docs/Module_0_Labs.md) before assembly.
4.  **Validation:** Run `python3 src/final_check.py` to verify your installation.

## 🛠 Hardware Platform
*   **Frame:** BetaFPV Pavo20
*   **Compute:** Raspberry Pi Zero 2 W (Quad-core Linux)
*   **Flight Controller:** STM32 F405 (Running Betaflight as an actuator server)
*   **Sensors:** 8x8 ToF Matrix, 1D Lidar, PMW3901 Optical Flow, M10Q GPS/Compass, IMX219 Vision.

## 🛡 Safety & Legal
See the [Safety Manual](docs/safety_manual.md) and [Module 0: Legal Compliance](docs/Module_0_The_Build.md) for FAA/EASA regulations. **PROPS OFF ON THE BENCH.**