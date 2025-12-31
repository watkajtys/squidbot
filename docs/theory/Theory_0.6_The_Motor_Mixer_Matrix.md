# Theory Deep Dive 0.6: The Motor Mixer Matrix
**"The Geometry of Force."**

Your PID controller outputs 4 "Desired Torques":
1.  **$T$:** Total Thrust (Up/Down).
2.  **$\tau_x$:** Roll Torque (Left/Right).
3.  **$\tau_y$:** Pitch Torque (Forward/Backward).
4.  **$\tau_z$:** Yaw Torque (Spin).

But your drone has 4 motors ($M_1, M_2, M_3, M_4$). How do you translate 4 torques into 4 motor speeds? You use the **Mixer Matrix**.

---

## **1. The Mix Equations**
For an "X" configuration drone:
*   $M_1 = T + \tau_x + \tau_y - \tau_z$
*   $M_2 = T - \tau_x + \tau_y + \tau_z$
*   $M_3 = T - \tau_x - \tau_y - \tau_z$
*   $M_4 = T + \tau_x - \tau_y + \tau_z$

*(Note: The +/- signs depend on the rotation direction of your specific motors!)*

---

## **2. Normalization & Clipping**
What if the PID asks for 110% power on Motor 1? 
*   The motor can't do it. If you just "clip" it to 100%, you lose the **Torque Balance**, and the drone will flip.
*   **The Pro Way:** You must scale **all** motors down proportionally to preserve the "Shape" of the command while staying within the hardware limits.

**Lab Task:** Implement the Mixer in `src/labs/lab_0_6_mixers.py` and verify it with a "Table Test."
