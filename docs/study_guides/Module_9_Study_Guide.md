# Study Guide 9: Path Planning and Search
**Module 9: The Intelligent Path**

### Critical Takeaways
1.  **Search Algorithms (A*):** The A-Star algorithm is the industry standard for discrete grid search. It uses a **Heuristic ($h$)**—a "guess" of the remaining distance—to prioritize which cells to explore. An "Admissible" heuristic (one that never overestimates the cost) guarantees that the algorithm will find the shortest possible path.
2.  **Sampling-Based Planners (RRT/RRT*):** In high-dimensional spaces where a grid is too computationally expensive, we use Rapidly-exploring Random Trees. RRT "samples" random points in space and connects them to build a "tree" of flyable paths. RRT* (RRT-Star) is an optimization that "rewires" the tree to find the most efficient path.
3.  **The Curse of Dimensionality:** A vacuum robot only needs to plan in 2D ($x, y$). A drone must plan in at least 4D ($x, y, z, \psi$), and potentially more if we consider velocity and acceleration limits. Every extra dimension exponentially increases the number of "cells" or "samples" required.
4.  **Trajectory Optimization:** A series of discrete points from a planner is not flyable by a drone—it would result in "jagged" movements that would damage the motors. We use **Minimum Snap** optimization to generate a smooth, polynomial curve that respects the physical limits of the drone.

### Mental Models and Field Notes
*   **The Accountant's Path:** Think of A* as an accountant. It tracks the "Money Spent" (distance already traveled, $g$) and the "Projected Debt" (distance remaining to goal, $h$). It always explores the path with the lowest "Total Cost" ($f = g + h$).
*   **Local vs. Global Planning:** A "Local" planner is like your "Instinct"—it helps you dodge a ball. A "Global" planner is like a "Map"—it tells you how to get to the next city. A drone needs both: Global planning to find the room, and Local planning to avoid the ceiling fan.
*   **The Voxel "Buffer":** Never plan a path exactly against a wall. Always "inflate" your obstacles in the map by the radius of the drone plus a safety margin. This ensures that even with a small control error, the drone won't clip the wall.

### Frontier Facts and Historical Context
*   **Bio-Inspired Navigation:** Bees navigate through forests by measuring "Optical Flow Balance." They don't build a 3D map; they simply try to keep the "Visual Speed" of objects on their left and right eyes equal.
*   **The Raven Challenge:** Find a high perch (shelf/cabinet) and land on it autonomously. This requires the "Raven" logic—identifying a small horizontal surface and planning a trajectory that "settles" onto it without bouncing.
*   **Deep-Space Planning:** The Mars Rovers (Curiosity/Perseverance) use a version of the A* algorithm called "D*-Lite." Because they move so slowly and the terrain changes (e.g., sand traps), they must constantly "re-plan" their path in real-time as they discover new obstacles.

---

### The Squid Games: Level 9
**The Maze Runner Challenge**
Generate a 2D "Artificial Maze" in Python and use your A* implementation to find the shortest path.
*   **The Goal:** Compare the number of nodes visited using "Manhattan Distance" vs "Euclidean Distance" heuristics.
*   **Win Condition:** Successfully planning a path through a 100x100 grid in less than 50ms.

---

### Module 9 Quiz
1.  **A* Heuristics:** Why must a heuristic be "Admissible"?
2.  **RRT:** Why is RRT considered "Probabilistically Complete"?
3.  **Optimization:** Why do we optimize for "Snap" (the 4th derivative of position) for quadrotors?
4.  **Collision Checking:** How do you mathematically determine if a straight-line path between two points in a 3D grid is "Clear" of obstacles?

---
*Reference: Lecture 9.5 (Path Planning and Search) in docs/LECTURES.md*
