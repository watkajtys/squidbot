# Module 8: Perception & Mapping
**"To fly blind is to fly scared."**

The drone knows *where* it is (Module 7). Now it needs to know *what* is around it. We will turn the 8x8 ToF Distance Matrix into a 3D map of the world.

---

## **8.1 Point Clouds & Voxels**

### **Objective**
Turn numbers into geometry.

### **Theory**
*   **Point Cloud:** A list of (X, Y, Z) dots.
*   **Voxel Grid:** A 3D Minecraft world. We divide the room into 10cm cubes.
    *   **Free:** Empty space.
    *   **Occupied:** Wall/Obstacle.
    *   **Unknown:** Areas we haven't seen.

### **Lab Procedure**
1.  **The Sensor:** The VL53L5CX gives an 8x8 grid of distances.
2.  **The Transform:** For each of the 64 points:
    *   Calculate local (x,y,z) based on the sensor's Field of View (45 deg).
    *   Rotate it by the Drone's Orientation (Quaternion from Mod 7).
    *   Add the Drone's Position.
3.  **Visualization:** Publish `sensor_msgs/PointCloud2` to RViz.

### **8.1.1 Sub-Lab: The Mirror World**
**"Lidar is not magic."**

1.  **Setup:** Place a large mirror in front of the drone.
2.  **Scan:** Point the Lidar at it.
3.  **Observe:** The Lidar beam bounces off the mirror and hits the wall behind you.
4.  **Result:** The map shows a "Ghost Room" extending *through* the mirror. This teaches you about "Multi-path Interference" and why we need ultrasonic or camera fusion to detect glass/mirrors.

### **Deliverable**
*   A screenshot of RViz showing a "Ghost Wall" appearing as you face the drone toward a real wall.

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
