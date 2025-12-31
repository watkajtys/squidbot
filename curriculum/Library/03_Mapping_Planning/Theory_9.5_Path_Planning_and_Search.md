# Theory Deep Dive 9.5: Path Planning & Search
**"Finding a way through the noise."**

In Module 9, you learned how to make a **smooth** path. But how do you find the path in the first place?

---

## **1. Discrete Search (A*)**
Imagine the room is a grid of 10cm cubes (Voxels).
*   **The Cost Function:** $f(n) = g(n) + h(n)$
    *   $g(n)$: Distance from start.
    *   $h(n)$: Estimated distance to goal (The Heuristic).
*   **A* (A-Star):** The industry standard for finding the shortest path on a grid.

---

## **2. Sampling-Based Planning (RRT*)**
What if the space is too big for a grid?
*   **Rapidly-exploring Random Trees (RRT):**
    1.  Pick a random point in the room.
    2.  Try to connect the drone to that point.
    3.  If there's an obstacle, stop.
    4.  Repeat until you hit the goal.
*   **RRT*:** An optimized version that "rewires" the tree to find the shortest possible path.

---

## **3. Obstacle Avoidance (Vector Field Histogram)**
How do you move when the obstacles are moving (like a person walking)?
*   **The "Magnetic" Model:** 
    *   The **Goal** has a "Positive Charge" (pulls the drone).
    *   **Obstacles** have a "Negative Charge" (pushes the drone away).
*   The drone simply follows the resulting **Vector Field**.

**Stanford Link:** See CS221 (Artificial Intelligence: Principles and Techniques) for the formal proofs of these algorithms.
--- [Return to Course Map](../../../COURSE_MAP.md)