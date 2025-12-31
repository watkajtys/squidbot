# The Squid: Autonomous Micro-Drone Project
**From Soldering Iron to Swarm Intelligence**

The Squid is a first-principles autonomous drone platform designed to bridge the gap between hobbyist flight and university-level robotics research. This project is structured as a full-stack robotics masterclass, taking the student from raw electronics to PhD-level Visual Inertial Odometry (VIO) and Neural Navigation.

## How This Stacks Up: The Missing Middle
This course was born from a specific frustration: the massive gap between hobbyist tutorials that treat drones as "black boxes" and academic papers that are inaccessible without a PhD. It was built by someone who wanted to get educated from first principles—not just to make a machine fly, but to understand the mathematics and architecture that make flight possible.

Squid fills this gap by providing a graduate-level curriculum on consumer-grade hardware. We take the rigorous theory found in elite research labs (ETH Zurich, MIT, Stanford) and apply it directly to the bench. We transition from 1960s-era classical control to 2025-era AI research—including **Liquid Neural Networks**, **3D Gaussian Splatting**, and **Hamilton-Jacobi Reachability**—in a single, coherent arc. This is the "Soldering Iron to Swarm" journey.

## The Mission: Systems Architecture
Most robotics education focuses on tool usage. The Squid Project focuses on tool creation. You will write your own drivers, derive your own control laws, and implement state-of-the-art perception. By the end of this course, you will not just be a developer; you will be a Systems Architect capable of working at the highest levels of aerospace and AI research.

## Project Structure
*   **[COURSE_MAP.md](COURSE_MAP.md)**: The high-level visual roadmap of the curriculum.
*   **[COURSE.md](COURSE.md)**: The integrated step-by-step experiment guide.
*   **curriculum/**: Centralized educational assets.
    *   **theory_deep_dives/**: Mathematical deep-dives into Jacobians and Lie Theory.
    *   **phase_resources/**: Study guides and mental models.
    *   **phase_1_mechanic/** to **phase_7_frontier/**: Module experiments.
*   **src/labs/**: 31 scaffolded labs organized by Phase subfolders.
*   **src/drivers/**: Production Python drivers for MSP, ToF, and Optical Flow.
*   **simulation/**: PyBullet SITL (Software-in-the-Loop) engine.
*   **tests/**: Mathematical validation suite.

## Getting Started
1.  **Prerequisites:** Read the **[SYLLABUS.md](SYLLABUS.md)** and **[Hardware Foundations](curriculum/support_tools/Hardware_Foundations.md)**.
2.  **Environment:** Follow the **[Pi Zero Survival Guide](curriculum/support_tools/PI_ZERO_SURVIVAL_GUIDE.md)** for initial setup.
3.  **Labs:** Start with **[Lab 0.1](src/labs/phase_1/lab_0_1_realtime_jitter.py)** to verify your timing budget.

## The Burnout Shield
This curriculum is academically dense. To ensure completion:
1.  **The Squid Games:** Treat every Phase as a game with a physical "Win" condition. Record a video of your success before diving back into the math.
2.  **The 80/20 Rule:** If the math in a Theory Deep Dive is making you want to quit, skip to the Lab. Real robots are built by doing, not just reading.
3.  **Fly for Fun:** Remind yourself why you are building this by flying in FPV mode weekly.

---
**Standard**: Academically aligned with M.Sc. Robotics / PhD Candidate Preparatory standards.
