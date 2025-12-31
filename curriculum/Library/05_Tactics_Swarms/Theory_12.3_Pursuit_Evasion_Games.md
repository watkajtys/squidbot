# Theory Deep Dive 12.3: Pursuit-Evasion Games
**"The Geometry of the Hunt."**

In dogfighting, you are playing a **Zero-Sum Game**. Every gain for the Pursuer is a loss for the Evader.

---

## **1. The Capture Zone**
A capture zone is a volume of state-space where the pursuer can guarantee a win.
*   **The HJI Equation:** The Hamilton-Jacobi-Isaacs equation is the game-theory version of the Bellman equation. It calculates the "Value" of every position in the room.
*   **The Win:** If you can keep the target inside your capture zone, the fight is over before it begins.

---

## **2. Apollonius Circles**
If two drones have different maximum speeds, the "meeting point" isn't the middle.
*   The set of all points where the pursuer and evader could meet at the same time forms a circle (The Apollonius Circle).
*   **Tactics:** As the pursuer, your goal is to shrink the Apollonius Circle until it contains only one point: the target.

---

## **3. The Dogfight Strategy**
*   **Pure Pursuit:** Points at the target's current position. (Beginner).
*   **Lead Pursuit:** Points at the target's *future* position. (Intermediate - Pro-Nav).
*   **Lag Pursuit:** Points *behind* the target to maintain energy and wait for a mistake. (Advanced).

**Stanford Link:** This mirrors research in the **Stanford Intelligent Systems Laboratory (SISL)** on collision avoidance and aircraft sequencing.
--- [Return to Course Map](../../../COURSE_MAP.md)