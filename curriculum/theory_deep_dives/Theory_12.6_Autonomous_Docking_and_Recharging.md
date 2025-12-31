# Theory Deep Dive 12.6: Autonomous Docking & Recharging
**"The Infinite Mission."**

A robot that can't charge itself is just a toy with a deadline.

---

## **1. The "Approach" Phase**
We use GPS/Lidar to get within 1 meter of the dock. But Lidar is too "coarse" for the final 10cm.

---

## **2. Visual Servoing (AprilTags)**
AprilTags are like QR codes designed for robots.
*   **The Math:** By looking at the distortion of the tag's square shape, the drone can calculate its **exact 3D distance and angle** relative to the dock (Pose Estimation).
*   **The Control:** The drone minimizes the "Visual Error" until it is perfectly centered.

---

## **3. The "Kill-Switch" Power State**
Once perched, the drone should:
1.  **Disarm** (Save battery).
2.  **Telemetry Off** (Switch to a low-power "Listen" mode).
3.  **Heartbeat Only:** Send a packet once every 10 seconds to say "I'm alive and charging."

**Hardware Tip:** Your dock should have a funnel-like shape to "catch" the drone if the landing is slightly off.
