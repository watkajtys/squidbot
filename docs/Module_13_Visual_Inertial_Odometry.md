# Module 13: Visual Inertial Odometry (VIO)
**"Flying without GPS."**

In this module, you will implement the most advanced form of robot navigation: **Visual Inertial Odometry**. This allows your drone to know its position by "looking" at the world and "feeling" its motion, even in total darkness (if using IR) or indoors where GPS fails.

---

## **13.1 Feature Tracking (The Eyes)**

### **Objective**
Find unique "landmarks" in the video feed and track them as the drone moves.

### **Theory**
We use the **Kanade-Lucas-Tomasi (KLT)** algorithm.
1.  **Detect:** Find "Features from Accelerated Segment Test" (FAST) corners.
2.  **Track:** For each feature, search the next frame for the most similar pixel neighborhood.
3.  **Optical Flow:** The displacement vector of these pixels tells us how the camera moved relative to the room.

### **Lab Procedure**
1.  **Code:** Create `src/vision/feature_tracker.py`.
2.  **Logic:** Use `cv2.goodFeaturesToTrack` and `cv2.calcOpticalFlowPyrLK`.
3.  **Visualization:** Draw green lines showing the "trajectories" of features on your dashboard.

---

## **13.2 IMU Pre-integration (The Motion)**

### **Objective**
Fuse 1000Hz IMU data into the 30Hz Vision pipeline.

### **Theory (Review Theory Deep Dive 6)**
We cannot run an optimizer at 1000Hz. We summarize 33 IMU readings into a single "Delta-Pose" between two camera frames.

### **Lab Procedure**
1.  **The Buffer:** Store IMU readings in a `deque`.
2.  **The Integration:** Use the Trapezoidal Rule to integrate Acceleration -> Velocity -> Position.
3.  **The Result:** A high-frequency estimate of where the drone is *between* camera frames.

---

## **13.3 The Factor Graph Solver**

### **Objective**
Find the most likely trajectory using Optimization.

### **Lab Procedure**
1.  **The Library:** We will use **GTSAM** (Georgia Tech Smoothing and Mapping).
2.  **The Graph:**
    *   **Prior Factor:** Initial position is (0,0,0).
    *   **IMU Factor:** Links Pose A to Pose B based on integrated motion.
    *   **Projection Factor:** Links Pose B to 3D landmarks seen in the camera.
3.  **Solve:** Run the Levenberg-Marquardt optimizer.

---

## **13.4 The Loop Closure (SLAM)**

### **Objective**
Snap the map back together.

### **Lab Procedure**
1.  **Bag of Words:** Use `DBoW2` to recognize if the current camera frame looks like a previous frame.
2.  **The Snap:** If a match is found, add a "Loop Closure Factor" to the GTSAM graph and re-solve.
3.  **Verify:** Fly a circle in your room. The estimated position should return to exactly zero when you reach the start.

---

## **Check: The Window Challenge**
**The PhD Graduation.**

1.  **Setup:** A hula hoop or a small window frame in the middle of the room.
2.  **The Task:** Fly the drone through the window using *only* VIO (GPS disconnected).
3.  **Constraint:** Perform a 360-degree rotation *before* passing through the window to test initialization robustness.

**Submission:** A plot of your VIO estimated path vs. ground truth (if available) or a video of the successful pass.
