# The Squid Drone Project
## From Soldering Iron to Swarm Intelligence

**Status:** Open Source Curriculum
**Level:** Graduate / Advanced Undergraduate
**Hardware Cost:** <$250 USD

---

### The Mission

Most drone tutorials focus on assembling off-the-shelf components and configuring pre-written software like ArduPilot or Betaflight. While valuable, this approach treats the drone as a "Black Box."

The Squid Project takes a different approach. We build an autonomous quadrotor from first principles. We write the low-level drivers, we derive the mathematical control laws on whiteboards, and we train the Neural Networks that guide flight.

The goal is not just to fly, but to understand the physics, mathematics, and software architecture that make flight possible. This curriculum mirrors the rigor of an Autonomous Mobile Robots course found at top engineering universities, bridging the gap between theory and real-world implementation.

---

### Core Competencies

By the end of this course, you will possess a "Full Stack" robotics skillset:

*   **Embedded Systems Engineering:** Understanding Real-time Linux constraints, writing I2C/UART drivers, and managing hardware interrupts.
*   **Control Systems Design:** Deriving PID controllers, analyzing stability margins, and performing physical System Identification.
*   **Signal Processing:** Implementing digital filters (Low-pass, Notch) and analyzing sensor noise using Allan Variance and FFTs.
*   **State Estimation:** fusing noisy sensor data (Lidar, IMU, Optical Flow) into a coherent state estimate using Extended Kalman Filters (EKF).
*   **Robotic Architecture:** Designing modular, scalable systems using ROS 2 (Robot Operating System) and Data Distribution Services (DDS).
*   **Artificial Intelligence:** Implementing Reinforcement Learning (PPO) pipelines, Sim-to-Real transfer strategies, and computer vision algorithms.

---

### Curriculum Roadmap

#### Phase I: The Mechanic (Hardware & Instincts)
*Building the machine and understanding its nervous system.*

*   **Module 0: The Build & Legal Compliance**
    Assembly of the custom Pavo20 frame, integrating a Raspberry Pi Zero 2 W with a Flight Controller. Comprehensive overview of FAA/EASA regulations, Remote ID, and safety protocols.

*   **Module 1: The Bare Metal API**
    Writing raw Python drivers to communicate with hardware. We use Logic Analyzers to verify DSHOT motor protocols at the microsecond level and implement direct register access for sensor management.

#### Phase II: The Test Pilot (Observability)
*Creating the tools required to visualize invisible data.*

*   **Module 2: The Telemetry Stack**
    Building a high-frequency, low-latency logging system using UDP sockets. Development of a React-based real-time dashboard to visualize flight data.

*   **Module 3: FPV & Augmented Reality**
    Constructing a low-latency video pipeline. We measure "Glass-to-Motor" latency using high-speed photography and overlay Lidar telemetry onto the video feed to create a pilot's Heads-Up Display (HUD).

#### Phase III: The Engineer (Control & Math)
*Taming the physics of flight through mathematics.*

*   **Module 4: Signal Processing & Geometry**
    A deep dive into sensor noise. We use Fast Fourier Transforms (FFT) to identify vibration frequencies and design Notch Filters to remove them. We also cover 3D coordinate transformations and Quaternions.

*   **Module 5: Control Theory**
    The heart of the autopilot. We derive the PID control law from calculus. We perform physical System Identification experiments, such as the Bifilar Pendulum test, to measure the drone's moment of inertia and tune the controller scientifically.

#### Phase IV: The Architect (Scale & Safety)
*Transitioning from scripts to a professional software architecture.*

*   **Module 6: The ROS 2 Migration**
    Porting our codebase to the industry-standard Robot Operating System (ROS 2). Implementation of "Watchdog" nodes and Fail-Safe State Machines to prevent flyaways during software crashes.

*   **Module 7: State Estimation (The Truth)**
    Implementing the Extended Kalman Filter (EKF). We derive the Jacobians necessary to fuse high-frequency IMU data with low-frequency Lidar and Optical Flow data, allowing the drone to know its position with millimeter precision.

#### Phase V: The Researcher (Autonomy)
*Teaching the drone to perceive and navigate the world.*

*   **Module 8: Perception & Mapping**
    Turning 1D sensor readings into 3D Volumetric Maps (Octomaps). We explore the physics of Lidar multipath interference (mirror reflections) and build point clouds in real-time.

*   **Module 9: Trajectory Optimization**
    Moving beyond simple point-to-point flight. We implement A* Pathfinding and smooth the results using Minimum Snap Splines. We also introduce Model Predictive Control (MPC) for handling complex dynamic constraints.

#### Phase VI: The Specialist (Tactics & AI)
*Deploying advanced intelligence and tactical behaviors.*

*   **Module 10: Reinforcement Learning**
    Training a Neural Network pilot in a physics simulation (Gym/PyBullet). We focus on Domain Randomization techniques to ensure the AI can fly the real drone despite physical discrepancies.

*   **Module 11: Aerial Combat & Guidance**
    Implementation of Proportional Navigation (Pro-Nav) guidance laws for intercepting moving targets. We build Visual Servoing systems to track and follow objects using computer vision.

*   **Module 12: Outdoor Autonomy**
    Integrating GPS for long-range navigation. We handle Coordinate Frame transformations (LLA to NED) and implement Behavior Trees to manage complex mission logic beyond simple state machines.

#### Phase VII: The Frontier (Experimental)
*PhD-level topics pushing the boundaries of micro-robotics.*

*   **Module 13: Visual Inertial Odometry (VIO)**
    Flying without GPS or Lidar. We explore the implementation of Factor Graphs and feature tracking to estimate position using only a monocular camera and an accelerometer.

*   **Module 14: Swarm Theory**
    Coordinating multiple drones. We configure ROS 2 for multi-agent communication and implement distributed consensus algorithms for formation flying.

---

### The Hardware Platform

The "Squid" is a custom-designed micro-drone optimized for indoor autonomy and educational accessibility.

*   **Frame:** BetaFPV Pavo20 (2-inch ducted CineWhoop)
*   **Compute:** Raspberry Pi Zero 2 W (Quad-core Linux, 512MB RAM)
*   **Flight Controller:** Betaflight F405 (STM32 microcontroller)
*   **Sensors:**
    *   **Mapping:** VL53L5CX (8x8 Zone Time-of-Flight Matrix)
    *   **Range:** 2x VL53L1X (1D Lidar for Altitude & Ceiling Lock)
    *   **Vision:** Arducam IMX219 (160-degree Fisheye Lens)
    *   **Optical Flow:** PMW3901 (Velocity estimation)
    *   **Position:** M10Q GPS + Compass (Outdoor navigation)

---

### Prerequisites

*   **Programming:** Intermediate Python experience. Familiarity with Object-Oriented Programming (Classes) is essential.
*   **Mathematics:** Basic Calculus (Derivatives/Integrals) and Linear Algebra (Matrix operations).
*   **Hardware:** Willingness to solder and debug electronic circuits.