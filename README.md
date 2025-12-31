# The Squid: Autonomous Micro-Drone Project
**From Soldering Iron to Swarm Intelligence**

The Squid is a first-principles autonomous drone platform designed to bridge the gap between hobbyist flight and university-level robotics research. This project is structured as a full-stack robotics masterclass, taking the student from raw electronics to PhD-level Visual Inertial Odometry (VIO) and Neural Navigation.

## How This Stacks Up: The Missing Middle
This course was born from a specific frustration: the massive gap between hobbyist tutorials that treat drones as "black boxes" and academic papers that are inaccessible without a PhD. It was built by someone who wanted to get educated from first principles—not just to make a machine fly, but to understand the mathematics and architecture that make flight possible.

Squid fills this gap by providing a graduate-level curriculum on consumer-grade hardware. We take the rigorous theory found in elite research labs (ETH Zurich, MIT, Stanford) and apply it directly to the bench. We transition from 1960s-era classical control to 2025-era AI research—including **Liquid Neural Networks**, **3D Gaussian Splatting**, and **Hamilton-Jacobi Reachability**—in a single, coherent arc. This is the "Soldering Iron to Swarm" journey.

## The Mission: Systems Architecture
Most robotics education focuses on tool usage. The Squid Project focuses on tool creation. You will write your own drivers, derive your own control laws, and implement state-of-the-art perception. By the end of this course, you will not just be a developer; you will be a Systems Architect capable of working at the highest levels of aerospace and AI research.

## The Roadmap (Phases I - VII)
1.  **Phase I: The Mechanic** (Hardware foundations, Linux scheduling, and raw Python drivers)
2.  **Phase II: The Test Pilot** (Telemetry architecture, MAVLink, and Augmented Reality HUDs)
3.  **Phase III: The Engineer** (Signal processing, Nyquist-Shannon, and PID Control Theory)
4.  **Phase IV: The Architect** (ROS 2 Migration, State Estimation, and Extended Kalman Filters)
5.  **Phase V: The Researcher** (Occupancy Mapping, Trajectory Optimization, and A-Star Search)
6.  **Phase VI: The Specialist** (Reinforcement Learning, Pro-Nav Guidance, and Tactical Autonomy)
7.  **Phase VII: The Frontier** (Visual Inertial Odometry, Swarm Consensus, and Deep Perception)

## Project Structure
*   **docs/**: 16 comprehensive modules, academic lectures, and high-density study guides.
    *   **study_guides/**: The core educational nodes with mental models and "Frontier Facts."
    *   **theory/**: Mathematical deep-dives into Jacobians, Factor Graphs, and Lie Theory.
    *   **LECTURES.md**: The theoretical study guide with academic reading lists.
    *   **COURSE.md**: The integrated step-by-step experiment guide.
*   **ros2_ws/**: A scaffolded ROS 2 (Humble/Iron) workspace for distributed drone intelligence.
*   **src/**: Production-ready Python source code for drivers, controllers, and laboratory experiments.
*   **simulation/**: PyBullet and Gym physics environments for "Sim-to-Real" validation.
*   **hardware/**: CAD files (.scad) and wiring schematics.
*   **tools/**: Automation scripts (e.g., setup_pi.sh).
*   **tests/**: A robust mathematical test suite to verify implementation logic before flight.

## Getting Started (Module 0)
1.  **Prerequisites:** Read [Hardware Foundations](docs/Hardware_Foundations.md) and [The Mathematician](docs/theory/Paper_0.1_The_Mathematician.md).
2.  **Environment:** Run `sudo ./tools/setup_pi.sh` on your Raspberry Pi Zero 2 W.
3.  **Labs:** Complete the [Module 0 Bench Labs](docs/Module_0_Labs.md) before assembly.
4.  **Validation:** Run `python3 src/final_check.py` to verify your installation.

## Hardware Platform
The Squid is a custom-designed micro-platform optimized for indoor autonomy and research:
*   **Frame**: BetaFPV Pavo20 (2-inch ducted CineWhoop).
*   **Compute**: Raspberry Pi Zero 2 W (Quad-core Linux, 512MB RAM).
*   **Flight Controller**: STM32 F405 (Running Betaflight as an actuator server).
*   **Sensors**:
    *   **Mapping**: VL53L5CX (8x8 Zone Time-of-Flight Matrix).
    *   **Range**: 2x VL53L1X (1D Lidar for Altitude and Ceiling Lock).
    *   **Vision**: Arducam IMX219 (160-degree Fisheye Lens).
    *   **Optical Flow**: PMW3901 (Velocity estimation).
    *   **Position**: M10Q GPS + Compass (Outdoor navigation).

## Safety and Legal
Robotics involves high-energy systems. Students must strictly adhere to the [Safety Manual](docs/safety_manual.md). Propellers must remain OFF during all bench-testing and driver development. All outdoor flight must comply with local FAA/EASA regulations regarding micro-UAVs (< 250g).

## The Burnout Shield
This curriculum is academically dense. To ensure completion:
1.  **The Squid Games:** Treat every Phase as a game with a physical "Win" condition. Record a video of your success before diving back into the math.
2.  **The 80/20 Rule:** If the math in a Theory Deep Dive is making you want to quit, skip to the Lab. Real robots are built by doing, not just reading.
3.  **Fly for Fun:** Remind yourself why you are building this by flying in FPV mode weekly.

---
**Standard**: Academically aligned with M.Sc. Robotics / PhD Candidate Preparatory standards.