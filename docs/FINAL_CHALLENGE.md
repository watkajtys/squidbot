# THE SQUID CAPSTONES: Final Graduation Missions
**Standard:** Systems Architect (Graduate Level)

Since this curriculum is self-directed, you may choose the Capstone that best aligns with your interests: Tactical Operations, Multi-Agent Systems, or AI Research. Completion of any one track grants "Squid Systems Architect" status.

---

## Path A: The Tactical Architect
**Focus:** Reliability, Control Theory, and Estimation.
**Designation:** Operation Deep Ink

**The Mission:** Map a GPS-denied, unstable basement and identify a hazardous leak.
1.  EKF Cold Start: Initialize with <1 degree gravity alignment.
2.  MPC Labyrinth: Navigate a U-shaped corridor at 0.2m/s without stopping or colliding.
3.  LNN Adaptation: Maintain altitude during a simulated 20% voltage sag (Battery Failure).
4.  3DGS Extraction: Perform a sub-centimeter precision landing on a 10cm pad.
*   **Best for:** Students interested in Aerospace Engineering and robust flight control.

---

## Path B: The Swarm Commander
**Focus:** Graph Theory, Consensus, and Distributed Systems.
**Designation:** Operation Hive Mind

**The Mission:** Coordinate a formation between one real Squid drone and two "Ghost" drones (Simulated/Virtual).
1.  Consensus Protocol: The real drone must maintain a perfect triangle formation with the Ghosts.
2.  Adversarial Tracking: One Ghost becomes "Rogue." Use an IMM Filter to predict its path and intercept it.
3.  Safety Barriers: While maneuvering, use CBFs to ensure the real drone never enters the "Safe Distance" buffer of any other agent.
4.  Distributed Landing: All three agents must land simultaneously on three different pads.
*   **Best for:** Students interested in Multi-Agent Systems and Robotics Networks.

---

## Path C: The AI Researcher
**Focus:** Reinforcement Learning, Sim-to-Real, and Differentiable Mapping.
**Designation:** Operation Neural Nest

**The Mission:** Navigate a complex room using only a pre-trained Neural Policy (No classical PID).
1.  Sim-to-Real Transfer: Train a PPO policy in the SITL Engine using domain randomization (varying mass/thrust).
2.  End-to-End Flight: Deploy the policy on the Pi Zero. The drone must navigate from Point A to Point B based only on ToF and IMU inputs.
3.  3DGS Semantic Perching: The drone must identify a "Perch" (a bookshelf or cabinet) and use a Gaussian Gradient to approach and land softly.
4.  Forensic Regression: Provide a NIS/NEES analysis proving the EKF stayed consistent throughout the neural-controlled flight.
*   **Best for:** Students interested in Deep Learning and the future of Autonomous Perception.

---

### Submission Requirements (Common to all Paths)
1.  The Log: A full MCAP or Foxglove log showing all internal state beliefs.
2.  The Tape: A continuous, uncut video of the flight achievement.
3.  The Code: A clean repository with all Lab TODOs implemented for your specific path.
4.  The Architect's Note: A brief (1-page) justification of the mathematical trade-offs you made.

---
**"First we prove it, then we build it, then we win it."**
**Congratulations, Pilot. The choice of the frontier is yours.**
