# Theory Deep Dive 14.4: Distributed Task Allocation
**"Who does what? The Swarm Playbook."**

In a dogfight, "3 drones chasing 1" is inefficient. You need roles.

---

## **1. The Assignment Problem**
Given $N$ drones and $M$ tasks (Intercept, Observe, Block), how do we pair them?
*   **The Global Solution:** One computer solves the "Hungarian Algorithm." (Slow, single point of failure).
*   **The Swarm Solution:** An **Auction**.

---

## **2. The Auction Algorithm**
1.  **Tasks** are announced to the swarm.
2.  **Drones** calculate a "Bid" based on:
    *   Distance to task.
    *   Battery remaining.
    *   Sensor health.
3.  The drone with the highest bid "wins" the task.

---

## **3. Emergent Behavior: The "Trap"**
By assigning different weights to tasks, you create "Emergent" strategy.
*   **Drone A:** High weight on "Closing Distance" (The Aggressor).
*   **Drone B:** High weight on "Cutting off Velocity Vector" (The Interceptor).
*   **Drone C:** High weight on "Staying 2m above the target" (The Overlook).
*   **Result:** Without ever talking to each other explicitly, the drones perform a coordinated "Pincer Movement."
--- [Return to Course Map](../../../COURSE_MAP.md)