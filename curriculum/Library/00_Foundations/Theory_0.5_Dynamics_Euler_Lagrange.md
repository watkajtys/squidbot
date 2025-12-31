# Theory Deep Dive 0.5: Euler-Lagrange Dynamics
**"From Energy to Equations."**

In Module 0.2, we used "Intuition." In this PhD-level dive, we use the **Lagrangian ($\mathcal{L}$)**.

---

## **1. The Energy Approach**
Instead of $F = ma$, we look at the Energy of the drone:
*   **Kinetic Energy ($T$):** Energy of motion (translational + rotational).
*   **Potential Energy ($V$):** Energy of position (gravity).
*   **The Lagrangian:** $\mathcal{L} = T - V$

---

## **2. The Equations of Motion**
The "Physics" of your drone is simply the solution to:
$$\frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}} \right) - \frac{\partial \mathcal{L}}{\partial q} = \tau$$
*   $q$: Your coordinates (X, Y, Z, Roll, Pitch, Yaw).
*   $\tau$: The external forces (Thrust from your motors).

**Why do this?**
When you reach **Module 9 (Trajectory Optimization)**, your computer needs these equations to predict exactly where the drone will be in 500ms. If your equations are wrong, your "Min-Snap" path will be unflyable.

**Exercise:** Try to derive the "Simple Pendulum" using this method before applying it to the 6-DOF Quadrotor.
--- [Return to Course Map](../../../COURSE_MAP.md)