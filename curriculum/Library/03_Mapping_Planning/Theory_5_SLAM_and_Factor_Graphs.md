# Theory Deep Dive 5: SLAM & Factor Graphs
**"Correcting the Past."**

In Module 8, you build a map. In a PhD-level system, you must accept that your map is always wrong and constantly fix it.

---

## **1. The Drift Problem**
Every sensor has a "Bias." If your IMU is off by 0.01 degrees, after 5 minutes of flying, your drone might think it is 10 meters away from where it actually is.
*   **Result:** Your map becomes "warped."

---

## **2. Loop Closure (The "Aha!" Moment)**
Imagine you fly from the Kitchen, through the Hallway, into the Living Room, and back to the Kitchen.
1.  **Odometry:** Says you are at `(10, 0, 0)`.
2.  **Vision:** Recognizes the specific pattern on the Kitchen wallpaper. It knows the Kitchen is at `(0, 0, 0)`.
3.  **The Conflict:** You have two "Truths" that don't match.

---

## **3. Factor Graphs**
In the old days, we used an EKF for this. In modern robotics, we use **Factor Graphs**.
*   **Nodes:** Your drone's position at different times ($x_1, x_2, x_3$).
*   **Factors:** Constraints between nodes.
    *   "I moved 1 meter" (Odometry Factor).
    *   "I am 2 meters from this wall" (Lidar Factor).
    *   "I am back at the start" (Loop Closure Factor).

### **Optimization**
Instead of a recursive filter, we solve a massive **Non-Linear Least Squares** problem. We "jiggle" all the positions in the graph until the "tension" (error) in all the factors is minimized.
*   **Tool:** You will use **GTSAM** or **Ceres Solver** for this.

---

## **4. Feature Matching: ORB & Bags of Words**
How does the drone "recognize" a room?
1.  **Features:** It finds high-contrast corners (ORB features).
2.  **Descriptors:** It turns those corners into a mathematical "fingerprint."
3.  **Bag of Words (BoW):** It keeps a "dictionary" of these fingerprints. When it sees a room that matches the dictionary, it triggers the Loop Closure.

**Study Task:** Look up "Graph-Based SLAM." It is the backbone of every self-driving car and autonomous drone.
--- [Return to Course Map](../../../COURSE_MAP.md)