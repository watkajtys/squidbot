# Theory Deep Dive 8.6: Semantic Mapping & Perch Detection
**"Beyond the Point Cloud."**

To perch, the drone must distinguish between "Noise" and a "Shelf."

---

## **1. Plane Fitting (RANSAC)**
How do we find a flat surface in a messy room? We use **Random Sample Consensus (RANSAC)**.
1.  Pick 3 random points from the Lidar data.
2.  Define a plane that passes through them.
3.  Count how many other points lie on that plane (the "Inliers").
4.  Repeat 100 times. The plane with the most inliers is your **Shelf**.

---

## **2. Normal Vector Analysis**
A "Perch" must be:
*   **Horizontal:** The "Normal Vector" must point straight up ($Z$).
*   **High:** The $Z$-coordinate must be $> 1.5m$ (above the floor).
*   **Clear:** There must be "Empty Space" (Voxels) above the plane so the drone doesn't hit the ceiling.

---

## **3. Semantic Labeling**
In your "Digital Twin" of your apartment, you will label these planes as `perch_point_1`, `perch_point_2`. Your **Path Planner (Module 9.5)** will then treat these as valid "Nodes" to fly to.
