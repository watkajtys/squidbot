[Previous Module](../../Phase_4_The_Architect/Module_07_5_Forensics/Module_07_5_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_09_Trajectory_Optimization/Module_09_Lecture.md)

---

# Module 8: Perception & Mapping
**"Seeing the world in 3D."**

## **8.1 The Point Cloud**

### **8.1.1 Sub-Lab: The Mirror Mystery**
**"Physics is a prankster."**

Lidars use light. Mirrors reflect light.

1.  **Test:** Point your drone's mapping sensor at a large mirror or a clean glass window.
2.  **Observe:** Look at your Foxglove/RVIZ visualization.
3.  **The Discovery:** You will see a "Ghost Room" extending *behind* the mirror. The Lidar doesn't know it hit a reflection; it thinks the light traveled twice as far into a new room.
4.  **The PhD Lesson:** This is called **Multipath Interference**. If your drone tries to fly into that "Ghost Room," it will hit the mirror. Professional SLAM systems use "Intensity" values to detect reflections.

---

## **8.2 Occupancy Mapping (Octomap)**

### **Objective**
Remember the map.

### **Theory**
A Point Cloud is instantaneous. If you turn away, the points disappear. An **Occupancy Map** has memory. It adds points to a global database.

### **Lab Procedure**
1.  **Install:** `sudo apt install ros-humble-octomap-server`.
2.  **Launch:** Configure `octomap_server` to listen to your PointCloud topic.
3.  **Scan:** Spin the drone 360 degrees in the center of the room.
4.  **Save:** `ros2 run octomap_server octomap_saver -f room_scan.bt`.

---

## **8.3 The Digital Twin Pipeline**

### **Objective**
Teleport your room into the Simulator.

This is the "Secret Sauce" of the Squid Project. We train AI in a simulation of *your actual house*.

### **Lab Procedure**
1.  **Export:** Convert the `.bt` (Octree) file to a `.urdf` or `.obj` mesh (using `octomap2obj`).
2.  **Import:** Load this mesh into `squid_drone/simulation/assets/`.
3.  **Physics:** Create a PyBullet collision shape.

### **Deliverable**
*   A side-by-side comparison: A photo of your room vs. the PyBullet simulation of your room.

---

## **Check: The Ghost Map**
**Reality Capture.**

1.  **Fly:** Manually fly the drone around your room (slowly!).
2.  **Build:** Watch the map build up in real-time on your laptop (RViz).
3.  **Verify:** Land. Check the map. Are the doorways open? Are the chairs visible?
4.  **Simulate:** Load the map into the Gym environment (Module 10 prep).

## **Theoretical Foundations**

### Lecture 8: Probabilistic Mapping & Graph SLAM

#### **1. The Map is a Hidden Markov Model (HMM)**
In modern robotics, we don't just "filter" the current position. We solve a massive **Non-Linear Least Squares** problem across the entire trajectory history.
*   **The Log-Odds Update:** We represent the world as a Voxel Grid. To update a voxel $n$, we use: $L(n|z_{1:k}) = L(n|z_{1:k-1}) + L(n|z_k)$. This additive approach is numerically faster and more stable than multiplying probabilities.
*   **Factor Graph Optimization:** We represent the drone's flight as a graph.
    *   **Nodes:** The Pose $x_i$ at time $t$.
    *   **Factors:** Constraints (e.g., "The IMU says I rotated $10^{\circ}$").
    *   **Optimization:** When we see a familiar room (**Loop Closure**), we "Snap" the graph back together by minimizing the sum of all factor errors simultaneously.

#### **2. Voxel Grids vs. Differentiable Radiance Fields**
*   **OctoMaps (Voxel Grids):** Uses a recursive tree structure to store occupancy. It is efficient for checking collision ($O(log N)$), but it is "Discrete" and "Blocky."
*   **3D Gaussian Splatting (3DGS):** The 2024 state-of-the-art. Instead of cubes, we use "Gaussians" (fuzzy blobs). Because Gaussians are differentiable, we can use **Gradient Descent** to find the "emptiest" path through a room—a feat impossible with standard point clouds.

### Lecture 8.6: Semantic Mapping & Surface Normals
Robots shouldn't just see "Shapes"; they should see "Semantics."
*   **Instance Segmentation:** We use a lightweight CNN (Module 15) to label voxels (e.g., "Shelf", "Floor").
*   **The Perch:** Once we identify a "Shelf," we calculate the **Surface Normal** $\hat{n}$ by finding the Eigenvectors of the local point cloud. The drone then computes a trajectory to align its landing gear with $\hat{n}$ for a soft landing.

**Next Step:** [Module 9: Trajectory Optimization](../Module_09_Trajectory_Optimization/Module_09_Lecture.md)

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"A robot without a map is a goldfish in a bowl. It has no memory of where it has been or where it can go. In this module, we give the drone a memory. We turn raw laser light into a permanent, 3D digital twin of your room. We are teaching our robot not just to 'see', but to 'remember' and 'understand' the geometry of its cage."

### **Deep Research Context: Multipath Interference**
In PhD-level research, mirrors are the "Final Boss." Lidars use time-of-flight. A mirror reflects the beam, making the Lidar think the room continues for meters into a "Ghost Room." Mention that we use **Pulse Intensity (RSSI)** to detect mirrors. If the return signal is very strong but the distance is very large, it's likely a reflection. This is how high-end vacuum robots avoid "falling" into a mirrored floor.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Diagram the Factor Graph representation of a SLAM trajectory.
- [ ] Explain the benefit of Loop Closure for correcting long-term odometry drift.
- [ ] Differentiate between a raw Point Cloud and a Probabilistic Voxel Grid (Octomap).
- [ ] Define "Surface Normals" and explain their role in autonomous perching.

---

## **Further Reading & Bibliography**

### **Mapping Systems**
*   **Hornung, A., et al. (2013).** *"OctoMap: An efficient probabilistic 3D mapping framework based on octrees."* Autonomous Robots. (The definitive OctoMap paper).
*   **Dellaert, F., & Kaess, M. (2006).** *"Square Root SAM: Simultaneous localization and mapping via square root information smoothing."* International Journal of Robotics Research.

### **The 3DGS Frontier**
*   **Kerbl, B., et al. (2023).** *"3D Gaussian Splatting for Real-Time Radiance Field Rendering."* SIGGRAPH. (The breakthrough paper).

---

[Previous Module](../../Phase_4_The_Architect/Module_07_5_Forensics/Module_07_5_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_09_Trajectory_Optimization/Module_09_Lecture.md)