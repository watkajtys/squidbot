# Theory Deep Dive 14.3: Control Barrier Functions (CBFs)
**"Provable Safety."**

A PID controller is a "Goal Seeker." It doesn't care if there is a wall in the way; it just wants to reach the target. A **Control Barrier Function (CBF)** is a "Guardian."

---

## **1. The Safety Set ($\mathcal{C}$)**
We define a mathematical "Safe Zone."
*   Example: $h(x) = dist\_to\_wall - 0.2m$
*   If $h(x) \geq 0$, the drone is safe.

---

## **2. The QP-Filter**
We take the "Desired" command from your PID and "Filter" it through a **Quadratic Program (QP)**.
*   **The Logic:** "Do whatever the PID says, **UNLESS** it violates the Safety Barrier ($h(x)$)."
*   If the PID says "Go Forward" but the barrier says "You are 21cm from the wall," the CBF will override the command and force the drone to stop.

---

## **3. Why this is "Elite"**
Standard drones use "If/Then" logic for safety.
*   *Hobby:* `if dist < 0.2: stop()`
*   *Elite (CBF):* The math ensures the drone **smoothly** approaches the boundary and "slides" along it. It is mathematically proven that the drone can never enter the unsafe state.
--- [Return to Course Map](../../../COURSE_MAP.md)