# The Squid Standard: Units, Frames & Conventions
**"Consistency is the first law of flight."**

In robotics, 90% of "unsolvable" bugs are actually just unit or coordinate frame mismatches. In this project, we follow the industry standards used by ROS 2 and NASA.

---

## **1. The Golden Units**
Unless a specific library requires otherwise, **ALWAYS** use these units in your code:
*   **Distance:** Meters (m)
*   **Angle:** Radians (rad) — *Exception: Use degrees ONLY for human-readable logs.*
*   **Velocity:** Meters per second (m/s)
*   **Angular Velocity:** Radians per second (rad/s)
*   **Time:** Seconds (s)
*   **Mass:** Kilograms (kg)

**The Rule:** `if "angle" in variable_name: check_radians()`

---

## **2. Coordinate Frames (REP-103)**
We follow the **Right-Hand Rule (RHR)**. 
Point your right thumb in the direction of the axis; your fingers curl in the **Positive** direction of rotation.

### **The Body Frame (The Drone)**
*   **X (Red):** Forward (The "Nose").
*   **Y (Green):** Left.
*   **Z (Blue):** Up.
*   *Memory Aid:* **RGB = XYZ**.

### **The World Frame (The Room)**
We use **ENU (East, North, Up)** for ROS 2 compatibility.
*   **X:** East.
*   **Y:** North.
*   **Z:** Up (Against Gravity).

---

## **3. Rotation Representation**
*   **Storage:** Always store rotations as **Quaternions** `[x, y, z, w]`.
*   **Math:** Convert to **Rotation Matrices** for vector multiplication.
*   **Human Interface:** Convert to **Euler Angles** (Roll, Pitch, Yaw) only for printing to the screen.

---

## **4. Data Flow Convention**
*   **Raw Data:** Unfiltered, straight from the sensor (e.g., `accel_raw`).
*   **Clean Data:** Filtered, bias-corrected, and converted to SI units (e.g., `accel_filtered`).
*   **State Estimate:** The "Truth" as calculated by your EKF (e.g., `pos_estimated`).

**Common Pitfall:** Never mix "Raw" and "Clean" data in the same mathematical expression.
--- [Return to Course Map](../../../COURSE_MAP.md)