# The Squid Projects: Systems Integration
**Level:** Graduate / Professional
**Goal:** Synthesis of multiple modules into a single functional capability.

---

## Project 1: The Labyrinth Navigator
**Modules:** 4, 5, 8, 9
**Hardware:** Pi Zero + VL53L5CX (ToF Array)
**Mission:** The drone is placed in a random unknown 2D maze (cardboard boxes). It must:
1.  **Map:** Rotate 360 degrees to build an initial Occupancy Grid.
2.  **Plan:** Find the exit using A* or RRT*.
3.  **Execute:** Navigate the path using **MPC Lite**, maintaining a 20cm buffer from all walls.
4.  **Win Condition:** Reach the exit without a single collision.

---

## Project 2: The Silent Guardian (Stealth & Stability)
**Modules:** 7, 10, 15
**Hardware:** Pi Zero + Arducam + PMW3901
**Mission:** Maintain a perfect hover in a dark room using only Optical Flow and IMU, while adapting to simulated physical changes.
1.  **Adapt:** During flight, a small weight is attached to one arm (simulated via software if necessary). The **Liquid Neural Network** must detect the imbalance and retune PID gains mid-air.
2.  **Localize:** Use **3D Gaussian Splatting** patches to recognize a "Perch" location and land autonomously.
3.  **Win Condition:** Hover drift < 5cm over 2 minutes; Landing accuracy < 3cm.

---

## Project 3: The Ghost in the Machine (Tactical Autonomy)
**Modules:** 11, 12, 14
**Hardware:** One Drone (Real) + One "Ghost" (Simulated)
**Mission:** A virtual "Enemy Drone" is projected into the ground station HUD. The real drone must intercept it.
1.  **Tracking:** Implement an **IMM Filter** to predict the erratic movement of the Ghost.
2.  **Guidance:** Use **Proportional Navigation (Pro-Nav)** to calculate the intercept trajectory.
3.  **Safety:** While intercepting, the **Control Barrier Function (CBF)** must ensure the real drone does not hit actual physical walls.
4.  **Win Condition:** Close within 30cm of the Ghost's virtual position while remaining > 20cm from real walls.

---

## Project 4: The Digital Twin (HIL Shadowing)
**Modules:** 3.5, 7, 13
**Hardware:** One Drone + SITL Engine
**Mission:** Run the real drone and the simulation in a "Parallel Reality" to detect sensor anomalies.
1.  **Sync:** Telemetry from the physical IMU is sent to the **SITL Engine** in real-time.
2.  **Shadow:** The simulation runs the "expected" physics based on motor commands.
3.  **Detect:** If the real EKF position deviates from the Shadow position by > 30cm, trigger an "Anomaly Alert" (simulating a motor failure or sensor glitch).
4.  **Win Condition:** Successfully identify a "Soft Failure" (e.g., covering one ToF sensor) by comparing the Shadow belief to the real belief.