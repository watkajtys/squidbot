# Theory Deep Dive 5.6: Nonlinear Control & LQR
**"Fighting the Curve."**

PID assumes that if you double the input, you double the output. This is a "Linear" assumption. But in drone flight, air resistance and gravity don't work that way.

---

## **1. The Linear Quadratic Regulator (LQR)**
Instead of tuning $P, I, D$ by hand, you define a **Cost Function** ($J$):
$$J = \int (x^T Q x + u^T R u) dt$$
*   **$Q$:** How much do I care about being on target?
*   **$R$:** How much do I care about saving battery/motor wear?
*   **The Computer:** Solves the math for you and gives you the "Optimal" gains.

---

## **2. Lyapunov Stability (The "Energy" View)**
How do you **prove** a drone won't crash?
Imagine a bowl. A marble at the top has "Potential Energy." As it rolls to the bottom, the energy decreases until it stops at the center.

**The Logic:**
If you can find a mathematical "Energy Function" ($V$) for your drone code that is **always decreasing**, your drone is **guaranteed** to be stable. This is how labs prove their code is safe before flying a $1M robot.

---

## **3. Underactuation (The "Acro" Problem)**
You have 4 motors but 6 things to control. 
*   **The Consequence:** To move forward, you *must* tilt down. You cannot move forward while staying perfectly level.
*   **The Math:** This is a "Constrained Optimization" problem. In Phase VI, you will use this to perform flips and rolls.
--- [Return to Course Map](../../../COURSE_MAP.md)