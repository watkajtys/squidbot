# Module 14: Swarm Theory
**"The Intelligence of the Crowd."**

In this final module, you move from one drone to a collective. We will explore how to make multiple drones behave like a single organism.

---

## **14.1 Distributed Consensus**

### **Objective**
Agreement without a leader.

### **Theory: The Laplacian Matrix**
How do birds stay together? Each bird follows three rules:
1.  **Cohesion:** Stay close to neighbors.
2.  **Separation:** Don't hit neighbors.
3.  **Alignment:** Fly in the same direction as neighbors.

---

## **14.2 Sub-Lab: The Consensus Drill**
**"Thinking as One."**

1.  **Setup (Simulation):** Launch 3 drones in `gym-pybullet-drones`.
2.  **Communication:** Each drone can only "see" its immediate neighbors (no global position).
3.  **The Algorithm:**
    *   `v_i = sum(pos_neighbors - pos_i)`
    *   This is the **Laplacian Feedback**. It "pulls" all drones toward the average position of the group.
4.  **The Test:** Give Drone 1 a new waypoint.
    *   **Success:** Drone 2 and 3 should follow Drone 1 automatically, maintaining a perfect formation, even though they were never told about the waypoint.

---

## **14.3 Collision Avoidance (CBF)**
**"The Force Field."**

We use **Control Barrier Functions (CBF)**. 
*   Think of each drone as having a "Magnetic Field" around it. 
*   If two drones get too close, the "Field" creates an unstoppable mathematical force that pushes them apart, overriding your mission code.

---

## **Check: The Synchronized Dance**
**The Graduation Ceremony.**

1.  **Scenario:** 3 drones in a line.
2.  **Task:** Perform a "Figure 8" where the drones must cross paths without colliding.
3.  **Constraint:** You can only control the "Swarm Center." The individual drone movements must be handled by the Laplacian math.

**Submission:** A simulation video showing the drones weaving through each other safely.
