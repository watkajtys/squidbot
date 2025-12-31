# Theory Deep Dive 11.5: Bayesian Search Theory
**"The Cat and Mouse Game."**

Before you can intercept, you must find.

---

## **1. The Probability Map (The Heatmap)**
Divide the room into a grid. Each cell has a value between 0.0 and 1.0 (The probability that the target is there).
*   **Prior:** "I think he is near the door."
*   **Update:** "I looked at the door and saw nothing."
*   **Result:** The probability near the door drops, and the probability everywhere else **rises**.

---

## **2. Probability of Detection ($P_d$)**
If you look at a cell, you aren't 100% sure the target isn't there. Maybe they were behind a chair.
*   **The Math:** $P_{new} = P_{old} \times (1 - P_d)$.
*   Even if you look, there is still a "shadow" of doubt.

---

## **3. Information-Optimal Trajectory**
Instead of flying a "lawnmower" pattern, the drone flies a path that **maximizes Information Gain**.
*   It flies to where the "uncertainty" is highest.
*   This turns searching from a chore into an **active hunt**.

**Stanford Link:** See CS228 (Probabilistic Graphical Models) for the math behind belief updates.
