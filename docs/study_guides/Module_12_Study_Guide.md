# Study Guide 12: Outdoor Autonomy
**Module 12: Beyond the Walls**

### Critical Takeaways
1.  **GNSS and Multipath:** Global Navigation Satellite Systems provide absolute position but are prone to **Multipath Interference**—where signals reflect off buildings before reaching the drone, causing "jumps" of several meters. We mitigate this using EKF fusion with high-frequency IMU data.
2.  **Magnetometer Calibration:** An IMU can tell you Pitch and Roll, but it cannot tell you absolute Heading (Yaw). For this, we use a Magnetometer. However, magnets are extremely sensitive to **Hard Iron** and **Soft Iron** interference. Calibration requires rotating the drone in a "Sphere" to map and cancel these local magnetic offsets.
3.  **Behavior Trees (BT):** For complex, multi-stage missions, we use Behavior Trees. BTs allow for hierarchical, modular logic. A mission to "Find and Land" is broken into: `Selector( Sequence(Find, Approach), Land)`. If the "Approach" fails, the BT can automatically "Fallback" to a "Search" behavior.
4.  **Precision Docking:** To land on a charging pad, GPS accuracy is insufficient. We use **Visual Markers (AprilTags)** or **Precision IR Beacons**. By detecting the unique geometry of a marker, the drone can calculate its relative pose to the landing pad with sub-centimeter accuracy.

### The Evolution of Navigation
*   **The Foundation (Magnetic & Dead Reckoning):** The "Old Tech" method of using a compass and measuring speed to guess your position. While it requires no satellites, the error accumulates so fast that the drone is lost within minutes.
*   **The Industry Standard (GNSS-IMU Fusion):** What you implement in `lab_7_ekf.py`. This uses satellites to keep the drone's position anchored to the world, while the IMU handles the fast, low-level movements.
*   **The Frontier (RTK-GPS & VIO):** Real-Time Kinematic GPS provides 2cm accuracy by measuring the phase of the satellite carrier wave. When combined with Visual Inertial Odometry, the drone can navigate with "Survey-Grade" precision.

### Mental Models and Field Notes
*   **The Invisible Compass:** Imagine trying to read a compass while sitting inside a metal cage surrounded by electric drills. That is the reality of a magnetometer on a micro-drone. You must mathematically subtract all the local "Noise" to find the "True North."
*   **The Mission Tree:** Think of a Behavior Tree like a flow chart that can "Think." It is constantly asking: "Am I succeeding?" If the answer is "No," it moves to the next logical branch, making the drone resilient to unexpected events.
*   **The Safety Bubble (Geofencing):** An outdoor drone must always have a "Virtual Cage." If the GPS coordinates cross a predefined boundary, the drone should immediately "Return to Home" (RTH). This is the most critical safety feature for outdoor flight.

### Frontier Facts and Historical Context
*   **RTK-GPS:** Traditional GPS has an error of 1-3 meters. RTK uses a fixed ground station to send "Correction Data" to the drone. By measuring the "Phase" of the satellite signal carrier wave, RTK allows for 2cm accuracy—enough to land on a moving vehicle.
*   **The Magnetic Field of Earth:** The Earth's magnetic field is actually quite weak (~0.5 Gauss). A single motor wire carrying 10 Amps of current can produce a field 10 times stronger than the Earth's. This is why "Soft Iron" calibration is the most difficult part of drone setup.
*   **Behavior Trees in Gaming:** Behavior Trees were originally popularized by the video game *Halo 2* to control AI enemies. Roboticists adopted them because they are much more flexible than standard "State Machines" for complex, unpredictable real-world missions.

---

### The Squid Games: Level 12
**The Precision Docking Challenge**
Using `lab_12_6_docking.py`, guide the drone to a landing pad using only a "Relative Vector" (e.g., as if it were seeing an AprilTag).
*   **The Goal:** Successfully land on the pad from a starting distance of 5 meters with a final position error of < 5cm.
*   **Win Condition:** A successful landing in under 20 seconds.

---

### Module 12 Quiz
1.  **Magnetometers:** Explain the difference between "Hard Iron" and "Soft Iron" interference.
2.  **Behavior Trees:** What is a "Blackboard" in the context of a Behavior Tree? Why is it useful for sharing data between nodes?
3.  **Geodetic Coordinates:** What is the difference between "LLA" (Latitude, Longitude, Altitude) and "NED" (North, East, Down)?
4.  **Safety:** Why should a drone never take off until it has a "GPS 3D Lock" with at least 6 satellites?

---
*Reference: Lecture 12.6 (Autonomous Docking and Recharging) in docs/LECTURES.md*
