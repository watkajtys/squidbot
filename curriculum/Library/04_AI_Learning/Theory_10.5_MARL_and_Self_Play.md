# Theory Deep Dive 10.5: MARL & Self-Play
**"The Evolution of Combat AI."**

You can't program every dogfighting tactic by hand. You must let the AI **discover** them.

---

## **1. Multi-Agent Reinforcement Learning (MARL)**
In single-agent RL, the environment is static. In MARL, the "environment" includes other drones that are also learning.
*   **The Challenge:** The "Moving Target" problem. As your teammate learns to help you, your old strategy might become useless.

---

## **2. Asymmetric Self-Play**
1.  **Agent A (The Hunter):** Gets points for staying within 1m of Agent B.
2.  **Agent B (The Evader):** Gets points for making Agent A lose lock.
3.  **The Result:** 
    *   Cycle 1: A flies in a line. B learns to zig-zag.
    *   Cycle 2: A learns to lead the turn. B learns to "over-shoot."
    *   Cycle 1000: Both agents have discovered **advanced tactical maneuvers** that a human never thought of.

---

## **3. Centralized Training, Decentralized Execution (CTDE)**
In your lab, the drones can share a "Big Brain" during training. But in the air, they must think for themselves.
*   **The Pro Way:** Use a **Critic** that sees everything during training, but an **Actor** that only sees the local drone's sensors during the fight.
--- [Return to Course Map](../../../COURSE_MAP.md)