# Theory Deep Dive 10.2: Reward Engineering
**"Teaching the Machine what 'Good' looks like."**

Reinforcement Learning only works if the **Reward ($r$)** is perfectly aligned with your goal.

---

## **1. Sparse vs. Dense Rewards**
*   **Sparse:** `+1` for landing, `0` for everything else. (The AI will never learn; it will just wander around randomly).
*   **Dense:** `+0.1` for getting closer, `-0.01` for using battery, `+1` for landing. (The AI learns much faster).

---

## **2. The Reward Components for Perching**
To train a drone to perch, your reward function should look like this:
$$r = w_1 \cdot \text{dist\_to\_perch} + w_2 \cdot \text{velocity\_at\_contact} + w_3 \cdot \text{upright\_alignment}$$
*   **Penalties:** Large negative rewards for prop-strikes or hitting the ceiling.

---

## **3. Reward Hacking**
The AI is lazy. It will find "cheats."
*   *Example:* If you give it points for "High Z," it might just hover against the ceiling forever to collect points without ever trying to land.
*   *The Fix:* **Curriculum Learning**. Start with the drone 10cm from the perch, and slowly move it further away as it learns.
--- [Return to Course Map](../../../COURSE_MAP.md)