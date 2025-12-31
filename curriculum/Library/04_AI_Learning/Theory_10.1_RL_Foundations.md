# Theory Deep Dive 10.1: RL for Robotics (Crash Course)
**"From Code to Curriculum."**

In standard robotics, you write: `if error > 0: do X`. 
In Reinforcement Learning, you write: `try something and see if you get a cookie.`

---

## **1. The MDP (Markov Decision Process)**
The "World" of an RL agent consists of:
*   **State ($s$):** What the drone sees (IMU, Lidar).
*   **Action ($a$):** What the drone does (Motor speeds).
*   **Reward ($r$):** The "Cookie" or the "Zapping."
*   **Policy ($\pi$):** The brain that decides $a$ based on $s$.

---

## **2. PPO: The Industry Standard**
We use **Proximal Policy Optimization (PPO)**. 
*   **Why:** It is "Stable." It prevents the AI from making a massive change to its brain that causes the drone to flip and die. It takes "small, safe steps" toward the goal.

---

## **3. The Training Loop**
1.  **Interact:** The drone flies in the Sim.
2.  **Collect:** We save the $(s, a, r)$ data.
3.  **Optimize:** We update the Neural Network to make the "High Reward" actions more likely.
4.  **Repeat:** Millions of times.
--- [Return to Course Map](../../../COURSE_MAP.md)