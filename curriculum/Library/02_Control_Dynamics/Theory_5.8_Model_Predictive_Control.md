# Theory Deep Dive 5.8: Model Predictive Control (MPC)
[Return to Module 5](../../Phases/Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md) | [Return to Course Map](../../../COURSE_MAP.md)

**"Thinking 5 Moves Ahead."**

PID and LQR are "Point-and-Shoot" controllers. **Model Predictive Control (MPC)** is a "Chess Grandmaster."

---

## **1. The Prediction Horizon ($N$)**
MPC doesn't just calculate the next motor command. It calculates the next **$N$** motor commands (e.g., the next 2 seconds of flight).
*   It asks: "What sequence of motor pulses will get me to the target while using the least energy and not hitting any walls?"

---

## **2. Constraints (The Killer App)**
This is why everyone uses MPC for real robots:
*   **PID:** Can't handle limits. It might ask for 200% throttle, which is impossible.
*   **MPC:** You explicitly add the rules: `0.0 <= throttle <= 1.0` and `dist_to_wall >= 0.2m`.
*   MPC will find the **mathematically fastest path** that obeys every single one of those rules.

---

## **3. The Receding Horizon**
1.  Solve the $N$-step optimization problem.
2.  Execute **only the first step**.
3.  Throw away the rest.
4.  Observe the new position and **Repeat**.

**The Downside:** Solving a $N$-step optimization problem 50 times a second is CPU-heavy. For the Pi Zero, we use "Linear MPC" to keep it fast.