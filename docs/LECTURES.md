# The "Squid" Lecture Series: From Zero to Autonomous Hunter
**Format:** High-Level Concepts -> Mathematical Derivation -> Implementation Details.

---

## **PART 1: THE FOUNDATIONS (Signal & Systems)**
*Theme: "The Robot is a Lie. Reality is Noise."*

### **Module 0: Signal Processing & The Physics of Sensors**
*   **Lecture 0.1: The Frequency Domain & Vibration**
    *   *Concept:* Why your drone shakes, and why you can't see it with the naked eye.
    *   *Math:* Fourier Series, Fast Fourier Transform (FFT), Sampling Theorem (Nyquist).
    *   *Deep Dive:* Designing Software Notch Filters vs. Low-Pass Filters (Delay vs. Cleanliness).
*   **Lecture 0.2: Camera Models & The Matrix**
    *   *Concept:* How a 3D world gets flattened onto a 2D sensor.
    *   *Math:* The Pinhole Camera Model, Intrinsic/Extrinsic Matrices, Radial Distortion coefficients.
    *   *Deep Dive:* The "Checkerboard" Calibration process and why "Fisheye" ruins navigation.
*   **Lecture 0.3: The Art of Timing**
    *   *Concept:* Time is not continuous in robotics.
    *   *Math:* Discrete Time Systems, Jitter, Latency distribution.
    *   *Deep Dive:* Analyzing Wi-Fi round-trip time and how to handle "Out-of-Order" UDP packets.

### **Module 1: Middleware & Architecture (ROS2)**
*   **Deep Dive:* Publish/Subscribe vs. Request/Response (Services) vs. Actions. When to use which?
*   **Lecture 1.2: Safety Critical Systems (FMEA)**
    *   *Concept:* Designing for failure.
    *   *Logic:* State Machines (Idle, Arming, Flight, Emergency).
    *   *Deep Dive:* Designing a "Watchdog" timer that kills the drone if the CPU freezes for 50ms.
*   **Lecture 1.3: Data Ops & The Black Box**
    *   *Concept:* If it isn't logged, it didn't happen.
    *   *Tech:* MCAP files, Rosbags, Foxglove Studio.
    *   *Deep Dive:* Managing 1TB of flight data. How to "Replay" a crash in simulation to fix the bug.

---

## **PART 2: STATE ESTIMATION (The Math of "Where am I?")**
*Theme: "You are never at [0,0]. You are at [0,0] with 5% uncertainty."*

### **Module 2: State Estimation & Sensor Fusion**
*   **Lecture 2.1: Probability & Belief**
    *   *Concept:* Robots don't know facts; they have "Belief Distributions."
    *   *Math:* Gaussian Distributions, Bayes Rule, Recursive Bayesian Estimation.
    *   *Deep Dive:* Why simple averaging fails when sensors disagree.
*   **Lecture 2.2: The Kalman Filter (Linear)**
    *   *Concept:* The optimal way to predict the future.
    *   *Math:* Prediction Step (Physics), Update Step (Measurement), Covariance Matrices.
    *   *Deep Dive:* Fusing a noisy accelerometer with a slow GPS (or Lidar).
*   **Lecture 2.3: The Extended Kalman Filter (EKF)**
    *   *Concept:* Reality isn't linear.
    *   *Math:* Taylor Series Expansion, Jacobians (The scary calculus part), Linearization.
    *   *Deep Dive:* Deriving the specific Jacobian matrix for the Pavo20's flight dynamics.
*   **Lecture 2.4: Quaternions & Rotations**
    *   *Concept:* Why Euler Angles (Roll/Pitch/Yaw) are broken (Gimbal Lock).
    *   *Math:* Complex Numbers, Hamilton product, SO(3) groups.
    *   *Deep Dive:* How to rotate a vector in 3D space without crying.

---

## **PART 3: PERCEPTION & MAPPING (The Digital Twin)**
*Theme: "Turning pixels into obstacles."*

### **Module 3: Mapping & Spatial Intelligence**
*   **Lecture 3.1: 3D Data Structures**
    *   *Concept:* How to store a room in RAM.
    *   *Tech:* Point Clouds, Voxel Grids, Octrees (Sparse Voxel Octrees).
    *   *Deep Dive:* Raycasting implementation for "Clearing" space (knowing where empty air is).
*   **Lecture 3.2: Occupancy Grids & Costmaps**
    *   *Concept:* Turning 3D shapes into a "Fly/No-Fly" map.
    *   *Math:* Log-Odds update rule (Probabilistic Mapping).
    *   *Deep Dive:* Inflation Radius (padding the walls so the drone doesn't graze them).
*   **Lecture 3.3: Alternative Senses (Novelty)**
    *   *Concept:* Seeing without eyes.
    *   *Research:* Tactile Sensing (Force feedback), Optical Flow divergence (Time-to-collision).
    *   *Deep Dive:* Implementing "Phototaxis" (Stealth/Shadow navigation).

---

## **PART 4: PLANNING & CONTROL (The Surgeon)**
*Theme: "Smooth is Fast."*

### **Module 4: Path Planning & Optimization**
*   **Lecture 4.1: Graph Search Algorithms**
    *   *Concept:* Finding the path through the maze.
    *   *Algo:* Dijkstra, A* (A-Star), JPS (Jump Point Search).
    *   *Deep Dive:* Heuristic functions for 3D flight (Euclidean vs. Manhattan distance).
*   **Lecture 4.2: Sampling-Based Planners**
    *   *Concept:* When the maze is too big to graph.
    *   *Algo:* RRT (Rapidly-exploring Random Trees), RRT*.
    *   *Deep Dive:* Finding a path in high-dimensional space.
*   **Lecture 4.3: Trajectory Optimization**
    *   *Concept:* A path is lines; a trajectory is time.
    *   *Math:* Polynomial Splines, Minimum Snap Optimization, QP (Quadratic Programming).
    *   *Deep Dive:* Generating a flight path that guarantees zero camera shake (minimizing Jerk).

---

## **PART 5: DYNAMICS & PHYSICS (The Acrobat)**
*Theme: "Cheating Gravity."*

### **Module 5: Advanced Aerodynamics & Dynamics**
*   **Lecture 5.1: Multirotor Physics**
    *   *Concept:* How propellers actually work.
    *   *Physics:* Thrust generation, Drag coefficients, Induced Velocity.
    *   *Deep Dive:* The "Ceiling Effect" and "Ground Effect" fluid dynamics.
*   **Lecture 5.2: System Identification**
    *   *Concept:* Measuring the "Black Box" of reality.
    *   *Experiment:* Chirp signals, Step responses, Bode Plots.
    *   *Deep Dive:* Creating a mathematical model of your drone to run in simulation.
*   **Lecture 5.3: Biomimetics**
    *   *Concept:* Stealing ideas from nature.
    *   *Research:* Perching (Birds), Optical Flow (Bees/Flies), Swarming (Starlings).
    *   *Deep Dive:* Implementing the "Ceiling Spider" perch.

---

## **PART 6: ARTIFICIAL INTELLIGENCE (The Brain)**
*Theme: "Learning what cannot be coded."*

### **Module 6: Reinforcement Learning (Sim-to-Real)**
*   **Lecture 6.1: The RL Framework**
    *   *Concept:* Agents, Environments, Rewards.
    *   *Math:* Markov Decision Processes (MDP), Value Functions.
    *   *Deep Dive:* Designing the "Reward Function" (Why AI finds loopholes).
*   **Lecture 6.2: Policy Gradients & PPO**
    *   *Concept:* How Neural Networks learn to move.
    *   *Algo:* Proximal Policy Optimization (The standard for robotics).
    *   *Deep Dive:* Actor-Critic architectures.
*   **Lecture 6.3: The Reality Gap**
    *   *Concept:* Why Sims lie.
    *   *Technique:* Domain Randomization, Dynamics Randomization.
    *   *Deep Dive:* Making the AI robust to wind, battery voltage drop, and lag.

---

## **PART 7: TACTICAL AUTONOMY (The Hunter)**
*Theme: "Win."*

### **Module 7: Combat & Interception Logic**
*   **Lecture 7.1: Guidance Laws**
    *   *Concept:* How missiles hit things.
    *   *Math:* Proportional Navigation (PN), Pure Pursuit, Deviated Pursuit.
    *   *Deep Dive:* Adapting missile math to a drone that can stop (Zero velocity).
*   **Lecture 7.2: Game Theory & Adversarial AI**
    *   *Concept:* When the environment fights back.
    *   *Math:* Minimax, Nash Equilibrium, Self-Play.
    *   *Deep Dive:* Training an AI to beat a copy of itself in a dogfight.
*   **Lecture 7.3: The Ethics of Autonomy**
    *   *Concept:* Just because we can build a Hunter-Killer, should we?
    *   *Discussion:* Rules of Engagement, Accountability in AI, Dual-Use Technology.

---

## **PART 8: THE HORIZON (Post-Grad)**
*Theme: "Beyond the single agent."*

### **Module 8: Future Frontiers**
*   **Lecture 8.1: Swarm Intelligence**
    *   *Concept:* Distributed Systems.
    *   *Research:* Consensus Algorithms (Raft/Paxos) for drones. Flocking (Boids).
    *   *Deep Dive:* Multi-Agent SLAM (Map Merging).
*   **Lecture 8.2: Visual Odometry (The Next Step)**
    *   *Concept:* Ditching the Lidar.
    *   *Algo:* ORB-SLAM3, VINS-Mono.
    *   *Deep Dive:* How to navigate outdoors with just a camera.
*   **Lecture 8.3: Custom Silicon**
    *   *Concept:* Optimization.
    *   *Hardware:* Designing a PCB with an STM32H7 and an FPGA.
    *   *Deep Dive:* Moving the Neural Network from Python to C++.
