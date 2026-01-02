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
    *   **Library/**: Mathematical deep-dives (EKF, MPC, Swarm Theory).
    *   **Phases/**: Module lectures and study guides organized by Phase 1-7.
    *   **Support/**: Hardware guides, setup tools, and debugging references.
        *   *[Emergency Quick Ref](curriculum/Support/Reference/QUICK_REF.md)*
*   **src/Labs/**: 31 scaffolded labs organized by Phase subfolders.
*   **src/drivers/**: Production Python drivers for MSP, ToF, and Optical Flow.
*   **simulation/**: PyBullet SITL (Software-in-the-Loop) engine.
*   **tests/**: Mathematical validation suite.

## Getting Started
1.  **Zero Hour:** Read **[PREFLIGHT.md](PREFLIGHT.md)** immediately. Do this *before* your hardware arrives. It includes the Simulator and Software Setup.
2.  **Prerequisites:** Read the **[SYLLABUS.md](SYLLABUS.md)** and **[Hardware Foundations](curriculum/Support/Hardware/Hardware_Foundations.md)**.
3.  **Environment:** Follow the **[Pi Zero Survival Guide](curriculum/Support/Setup/PI_ZERO_SURVIVAL_GUIDE.md)** for initial setup.
4.  **Labs:** Start with **[Lab 0.1](src/Labs/Phase_1_The_Mechanic/Module_00_The_Build/lab_0_1_realtime_jitter.py)** to verify your timing budget.

## The Burnout Shield
This curriculum is academically dense. To ensure completion:
1.  **The Squid Games:** Treat every Phase as a game with a physical "Win" condition. Record a video of your success before diving back into the math.
2.  **The 80/20 Rule:** If the math in a Theory Deep Dive is making you want to quit, skip to the Lab. Real robots are built by doing, not just reading.
3.  **Fly for Fun:** Remind yourself why you are building this by flying in FPV mode weekly.

## The Promise
You are about to start with a bag of loose parts and a blank text editor. You will end with a machine that has a mind of its own.

You will not just learn Python; you will learn how to make Python talk to the real world. You will not just learn Math; you will learn how to make Math move. The path is steep, and things will break. But the moment your code lifts that machine off the desk for the first time... there is no feeling like it in the world.

**You are ready.**

---
**Standard**: Academically aligned with M.Sc. Robotics / PhD Candidate Preparatory standards.
