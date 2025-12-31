# Theory Deep Dive 14.5: Multi-Agent Avoidance (RVO)
**"Decentralized Safety in a Dogfight."**

When 3 drones are chasing a target, the biggest risk isn't the target—it's the other drones. You cannot rely on a central computer to tell everyone where to go; it's too slow.

---

## **1. The Velocity Obstacle (VO)**
Imagine you are a drone. Every other drone creates a "Collision Cone" in your velocity space.
*   If you pick a velocity vector that points *inside* that cone, you will collide.
*   **The Fix:** You pick the "closest" velocity to your desired goal that stays *outside* the cone.

---

## **2. Reciprocal Velocity Obstacles (RVO)**
If two drones both move to avoid each other, they might "dance" back and forth (Oscillation).
*   **The Logic:** "I will assume the other drone will do 50% of the work to avoid me."
*   **The Result:** Both drones make a small, consistent adjustment. The swarm flows smoothly like a liquid.

---

## **3. ORCA (Optimal Reciprocal Collision Avoidance)**
ORCA is the high-performance version of RVO used in modern swarms.
*   It uses **Linear Programming** to find the safest velocity in microseconds.
*   It is mathematically guaranteed to be **collision-free** for any number of agents.

**Implementation Hint:** Use the `math_utils.py` to calculate the "Collision Cone" angles.
--- [Return to Course Map](../../../COURSE_MAP.md)