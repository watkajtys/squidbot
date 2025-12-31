# Module 8: Perception and Mapping
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

**Submission:** The `.obj` file of your scanned flight arena.
