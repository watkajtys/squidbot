# Squid Drone Project

## Overview

The Squid Drone project is an advanced, full-stack robotics curriculum designed to take students from hardware assembly to autonomous swarm intelligence. It utilizes a custom micro-drone platform (Raspberry Pi Zero 2 W + Betaflight) and a progressive learning path covering control theory, state estimation, and modern AI/RL techniques.

**Key Philosophy:** "First we prove it (Math), then we build it (Code), then we win it (Challenge)."

## Architecture & Tech Stack

The project is a hybrid of embedded Python development and ROS 2 architecture.

*   **Language:** Python 3 (primary), C++ (optional within ROS 2 nodes).
*   **Hardware:** Raspberry Pi Zero 2 W, STM32 Flight Controller (Betaflight), VL53L5CX ToF, PMW3901 Optical Flow, IMX219 Camera.
*   **Middleware:** ROS 2 (Humble/Iron compatible) for higher-level architecture.
*   **Simulation:** PyBullet (`gym-pybullet-drones`) for Software-in-the-Loop (SITL) testing.
*   **AI/ML:** Stable Baselines 3 (RL), TensorFlow Lite / OnnxRuntime (Edge AI), GTSAM (Factor Graphs/VIO).
*   **Data Analysis:** Foxglove Studio, PlotJuggler.

## Directory Structure

*   **`src/`**: The core codebase.
    *   **`drivers/`**: Low-level hardware drivers (MSP, ToF, Optical Flow).
    *   **`Labs/`**: Progressive educational modules (Phase 1 - 7).
    *   **`utils/`**: Helper functions for math, logging, and transforms.
    *   **`main.py`**: The main entry point for the "Bare Metal" flight loop.
*   **`curriculum/`**: Educational content.
    *   **`Library/`**: Theoretical deep-dives (Math, Physics, Algorithms).
    *   **`Phases/`**: Lecture notes and study guides.
    *   **`Support/`**: Hardware guides and setup references.
*   **`ros2_ws/`**: The ROS 2 workspace containing the `squid_control` package.
*   **`simulation/`**: PyBullet simulation assets (`quadrotor.urdf`) and engines.
*   **`tests/`**: Unit and regression tests.
*   **`tools/`**: Utility scripts for data simulation, regression testing, and environment setup.

## Development Workflow

1.  **Phase-Based Learning:** The project is divided into 7 Phases. Work typically proceeds by reading the `Module_XX_Study_Guide.md` in `curriculum/Phases/` and then completing the corresponding lab in `src/Labs/`.
2.  **Simulation First:** Complex logic (Control, RL) is often tested in the PyBullet simulator (`simulation/`) using the `quadrotor.urdf` model before deployment.
3.  **Bare Metal vs. ROS 2:**
    *   Early phases focus on "Bare Metal" Python scripts in `src/`.
    *   Later phases migrate to a distributed ROS 2 architecture in `ros2_ws/`.
4.  **Verification:**
    *   **Unit Tests:** `pytest tests/`
    *   **Regression:** `python tools/regression_test.py`
    *   **Challenges:** Each phase has a "Squid Game" physical challenge to verify functionality (see `SQUID_GAMES.md`).

## Key Commands

*   **Install Dependencies:** `pip install -r requirements.txt`
*   **Run Main Flight Loop:** `python src/main.py` (Ensure hardware is connected or mocks are in place)
*   **Run Tests:** `pytest`
*   **Build ROS 2 Package:**
    ```bash
    cd ros2_ws
    colcon build --symlink-install
    source install/setup.bash
    ```
*   **Run Simulation:** `python simulation/sim_engine.py` (or specific lab scripts)

## Documentation

*   **`README.md`**: High-level introduction.
*   **`SYLLABUS.md`**: Detailed curriculum roadmap.
*   **`COURSE_MAP.md`**: Visual guide to the course.
*   **`PREFLIGHT.md`**: Initial setup and safety checks.
*   **`SQUID_GAMES.md`**: Physical challenges and milestones.
