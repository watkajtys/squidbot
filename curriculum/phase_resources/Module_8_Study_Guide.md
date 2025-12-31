# Study Guide 8: Perception and Mapping
**Module 8: The Machine's Eye**

### Critical Takeaways
1.  **The Occupancy Grid Mapping (OGM):** To navigate, a drone must divide a continuous 3D world into discrete cells (Voxels). Each cell contains a probability of being "Occupied," "Free," or "Unknown." This allows the drone to differentiate between "Empty Air" and "Unexplored Space."
2.  **Ray-Casting Logic:** When a Lidar or ToF sensor report a hit at 2.0 meters, the mapping algorithm performs "Ray-Casting." It marks the cell at 2.0m as Occupied, and *every cell between the sensor and 2.0m* as Free. This is how a robot "clears" a room.
3.  **Simultaneous Localization and Mapping (SLAM):** The foundational challenge of mobile robotics. To build a map, you need your position; to get your position, you need a map. We solve this "Chicken-and-Egg" problem using **Factor Graphs**, where Poses and Landmarks are nodes connected by measurement constraints.
4.  **Log-Odds Notation:** We never store raw probabilities (e.g., 0.95). Instead, we store the Log-Odds ($L = \log \frac{P}{1-P}$). This allows us to perform map updates using simple Addition ($L_{new} = L_{old} + L_{sensor}$) instead of slow, numerically unstable Multiplications.

### Mental Models and Field Notes
*   **The Fog of War:** Think of an unexplored room as being filled with a thick grey fog. As the drone's sensors "touch" the world, they carve out tunnels of "Free" space through that fog. If the sensor hits a wall, it leaves a "Permanent Mark" in the fog. Mapping is the art of clearing the fog.
*   **The Voxel Cost:** To a robot, a wall is not an object; it is an "Infinite Cost." A path planner is simply an accountant trying to find the "Cheapest" way through a room without going bankrupt (colliding).
*   **Drift is the Enemy:** If your state estimate drifts by 10cm, your map will also be blurry by 10cm. A "Sharp" map is the best proof of a well-tuned EKF.

### Frontier Facts and Historical Context
*   **Semantic Mapping:** Current research is moving beyond "Occupancy." "Semantic SLAM" allows a drone to recognize *what* it is seeing. Instead of just an "Obstacle," the map says "Chair," "Doorway," or "Fire Extinguisher."
*   **3D Gaussian Splatting (3DGS):** This represents the world as millions of tiny ellipsoids. Because Gaussians are differentiable, a drone can "render" a predicted view and compare it to its actual camera feed to calculate its position with sub-centimeter accuracy—even in featureless environments.
*   **The "Mirror Maze":** In mapping, mirrors are a nightmare. Because light bounces off them, the drone's Lidar thinks the room continues "inside" the mirror. In the field, we use "Multipath Rejection" logic to identify these "Ghost Rooms."

---

### The Squid Games: Level 8
**The Mirror Maze Challenge**
Fly the drone in a room containing a large floor-to-ceiling mirror.
*   **The Goal:** Observe your map. Does it show a "Ghost Room" behind the mirror?
*   **Win Condition:** Your mapping algorithm correctly identifies the mirror as a boundary (after multiple passes) and doesn't try to plan a path through it.

---

### Module 8 Quiz
1.  **Log-Odds:** If a cell has a probability of 0.5 (Unknown), what is its Log-Odds value?
2.  **Point Cloud Processing:** What is a "Voxel Grid Downsampling" filter, and why is it mandatory for processing data?
3.  **Bresenham's Algorithm:** Briefly explain how this algorithm is used in 3D Ray-Casting.
4.  **Closing the Loop:** Why does recognizing a previously visited landmark (Loop Closure) allow the SLAM algorithm to correct the error in the *entire* history of the flight?

---
*Reference: Lecture 9 (SLAM and Geometry) in docs/LECTURES.md*
