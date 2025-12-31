# Theory Deep Dive: Coordinate Systems & Quaternions
**"Which way is Up?"**

In Module 1, you spin a motor. In Module 4, you must rotate the drone in 3D space. This requires understanding the geometry of rotation.

---

## **1. The Body vs. The World**
Imagine you are sitting in the drone's cockpit.
*   **Body Frame ($B$):** Forward is X, Left is Y, Up is Z. This moves with you.
*   **World Frame ($W$):** North is X, East is Y, Down is Z (usually "NED" in aviation).

To translate a sensor reading (like the Accelerometer) from the Body to the World, we use a **Rotation Matrix ($R$)**.
$$ V_{world} = R \cdot V_{body} $$

---

## **2. The Euler Problem (Gimbal Lock)**
Most beginners use **Roll, Pitch, and Yaw**.
*   **The Danger:** If you pitch the drone exactly 90 degrees down, the Roll and Yaw axes align. The math "collapses" because it can no longer tell the difference between rolling and yawing. This is **Gimbal Lock**.

---

## **3. The Solution: Quaternions**
Robots use **Quaternions** ($q = [w, x, y, z]$). They are 4D vectors that represent 3D rotations without ever hitting a singularity (Gimbal Lock).

### **How to think about them:**
Don't try to visualize 4D space. Think of a Quaternion as:
1.  **An Axis:** A line in 3D space.
2.  **An Angle:** How much to twist around that line.

### **The Math You Need (Python):**
You don't need to derive them, but you must know how to use them with `scipy`.

```python
from scipy.spatial.transform import Rotation as R

# Create a rotation of 45 degrees around the Z axis
r = R.from_euler('z', 45, degrees=True)

# Convert to a Matrix (for math)
matrix = r.as_matrix()

# Convert to a Quaternion (for ROS 2 / EKF)
quat = r.as_quat() # Returns [x, y, z, w]
```

---

## **4. The Transformation Tree (TF2)**
In Module 6 (ROS 2), you will use **TF2**. It is a database that keeps track of where every part of the drone is.
*   `base_link` -> `camera_link` (Offset by 3cm)
*   `base_link` -> `imu_link` (Offset by 0cm)
*   `world` -> `base_link` (The drone's actual position)

**Research Task:** Look up "Passive vs Active Rotations." It is the #1 cause of bugs in robotics.
