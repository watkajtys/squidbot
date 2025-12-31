# Mini-Lecture 0.2: The Physics of Flight
**"How 4 Spinning Sticks defy Gravity."**

A quadrotor is a "Non-Linear, Underactuated System." This means you have 6 things to control (X, Y, Z, Roll, Pitch, Yaw) but only 4 motors to do it.

---

## **1. The "X" Configuration**
Look at your Squid Drone from the top. The motors are numbered 1-4.
*   **Motors 1 & 3:** Spin Counter-Clockwise (CCW).
*   **Motors 2 & 4:** Spin Clockwise (CW).

**Why?** Because of Newton's 3rd Law: *Every action has an equal and opposite reaction.* 
*   If all props spun CW, the drone body would spin CCW like a top. By spinning them in opposite pairs, the "Torque" cancels out, and the drone stays still.

---

## **2. The 4 Basic Maneuvers**

### **A. Throttle (Going Up/Down)**
Increase the speed of **ALL 4** motors equally.
*   *Physics:* Total Thrust > Gravity.

### **B. Pitch (Moving Forward/Backward)**
To move Forward (Nose Down):
*   Increase speed of **Rear** motors (2 & 3).
*   Decrease speed of **Front** motors (1 & 4).
*   *Physics:* The rear lifts, the front drops, and the thrust vector points backward, pushing you forward.

### **C. Roll (Moving Left/Right)**
To move Left (Left Wing Down):
*   Increase speed of **Right** motors (1 & 2).
*   Decrease speed of **Left** motors (3 & 4).

### **D. Yaw (Spinning in Place)**
This is the "Magic" one. To spin Right:
*   Increase the **CCW** motors (1 & 3).
*   Decrease the **CW** motors (2 & 4).
*   *Physics:* The torque no longer cancels out. The "Reaction Torque" from the faster CCW props pushes the drone body CW (Right).

---

## **3. The "Hover" State**
Hovering is a dynamic act of balance. 
*   **Reality:** Your drone is never perfectly balanced. One motor is always slightly stronger, or the battery is 1mm off-center.
*   **The Job:** Your **PID Controller** (Module 5) will be adjusting these 4 motor speeds hundreds of times per second just to stay still.

**Student Challenge:** If Motor 1 fails mid-flight, can the drone still fly? 
*(Answer: No. A quadrotor needs all 4 to maintain torque balance. This is why "Hexacopters" exist!)*
