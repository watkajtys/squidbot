# Module 6: The ROS2 Migration
**"Welcome to the Industry Standard."**

Up to this point, we have written "Bare Metal" Python scripts. This works for simple projects, but it doesn't scale. If you want 10 different algorithms to talk to each other without crashing, you need a Middleware. We use **Robot Operating System 2 (ROS2)**.

---

## **6.1 Architecture: Nodes & Topics**

### **Objective**
Stop writing monolithic `while True` loops. Start writing "Nodes."

### **Theory**
*   **The Node:** A single process that does ONE thing well (e.g., `lidar_driver`, `pid_controller`).
*   **The Topic:** A named bus where nodes publish data (e.g., `/sensors/lidar`, `/control/cmd_vel`).
*   **DDS (Data Distribution Service):** The magic under the hood that moves data between nodes (even over WiFi).

### **Lab Procedure**
1.  **Setup:** Install ROS2 Humble on your Pi (or use the provided Docker container).
2.  **The Graph:** Draw your system.
    *   `tof_node` (Publishes: `Range`)
    *   `imu_node` (Publishes: `Imu`)
    *   `mixer_node` (Subscribes: `Roll/Pitch/Yaw`, Publishes: `Actuators`)

### **Deliverable**
*   A `rqt_graph` screenshot showing your planned architecture.

---

## **6.2 The Port**

### **Objective**
Refactor Module 1 & 5 code into ROS2 Classes.

### **Lab Procedure**
1.  **Create Package:** `ros2 pkg create --build-type ament_python squid_control`
2.  **Port the Driver:**
    *   Take your `ToFArray` class from Module 1.
    *   Wrap it in a `rclpy.Node`.
    *   Create a Timer (50Hz) to call `read_distances()` and publish a `sensor_msgs/Range` message.
3.  **Port the Controller:**
    *   Take your `PID` class from Module 5.
    *   Create a Subscriber for `/sensors/range`.
    *   In the callback, run the PID update and publish to `/cmd_motor`.

### **Deliverable**
*   A working ROS2 launch file that starts the driver and controller.
*   `ros2 topic echo /cmd_motor` showing live data.

---

## **6.3 Data Ops: MCAP Logging**

### **Objective**
Replace our CSV logger (Module 2) with the professional standard.

### **Theory**
*   **Rosbags (.mcap):** High-performance binary logging format. It records *everything* on the bus.
*   **Foxglove Studio:** The visualization tool used by Tesla/NVIDIA (and us).

### **Lab Procedure**
1.  **Record:** `ros2 bag record -a` while flying.
2.  **Visualize:** Drag the `.mcap` file into [Foxglove Studio](https://foxglove.dev/).
3.  **Analyze:** Plot your PID error and Motor Output on the same timeline.

---

## **6.4 Safety Architecture: The Watchdog**

### **Objective**
Prevent flyaways when code crashes.

### **Lab Procedure**
1.  **The Heartbeat:** Create a `watchdog_node` that listens for a 10Hz heartbeat from the `controller_node`.
2.  **The Kill Switch:** If the heartbeat stops for >0.5s (5 missing pulses), the Watchdog publishes `cmd_motor = 0` (or `land_mode`) directly to the microcontroller gatekeeper.
3.  **Test:** Intentionally `kill -9` your controller node mid-flight (simulated first!).

---

## **Check: The Replica**
**Does it still fly?**

The goal is Feature Parity. The drone should fly *exactly* as well as it did in Module 5, but now the code is modular.

1.  **Test:** Repeat the "Hover Test" (Module 5).
2.  **Verify:** If the drone jitters, check your topic frequency (`ros2 topic hz /sensors/imu`). Latency in ROS2 can be tricky!

**Submission:** A screenshot of Foxglove Studio displaying your flight data.
