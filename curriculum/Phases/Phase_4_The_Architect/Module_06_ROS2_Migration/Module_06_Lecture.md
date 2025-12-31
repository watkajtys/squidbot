[Previous Module](../../Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_06_5_Reliability/Module_06_5_Lecture.md)

---

# Module 6: The ROS 2 Migration
**"Welcome to the Industry Standard."**

Up to this point, we have written "Bare Metal" Python scripts. This works for simple projects, but it doesn't scale. If you want 10 different algorithms to talk to each other without crashing, you need a Middleware. We use **Robot Operating System 2 (ROS 2)**.

---

## **6.1 Architecture: Nodes & Topics**

### **Objective**
Stop writing monolithic `while True` loops. Start writing "Nodes."

### **Theory**
*   **The Node:** A single process that does ONE thing well (e.g., `lidar_driver`, `pid_controller`).
*   **The Topic:** A named bus where nodes publish data (e.g., `/sensors/lidar`, `/control/cmd_vel`).
*   **DDS (Data Distribution Service):** The magic under the hood that moves data between nodes (even over WiFi).

### **Lab Procedure**
1.  **Setup:** Install ROS 2 Humble on your Pi (or use the provided Docker container).
2.  **The Graph:** Draw your system.
    *   `tof_node` (Publishes: `Range`)
    *   `imu_node` (Publishes: `Imu`)
    *   `mixer_node` (Subscribes: `Roll/Pitch/Yaw`, Publishes: `Actuators`)

### **Deliverable**
*   A `rqt_graph` screenshot showing your planned architecture.

---

## **6.2 The Port**

### **Objective**
Refactor Module 1 & 5 code into ROS 2 Classes.

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
*   A working ROS 2 launch file that starts the driver and controller.
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

## **6.4 Safety Architecture: Lifecycle Nodes**

### **Objective**
Professional Safety Management.

### **Theory**
A simple node starts publishing as soon as you run it. This is dangerous. What if the sensors aren't ready? What if the calibration is wrong?
**ROS 2 Lifecycle Nodes (Managed Nodes)** implement a state machine:
1.  **Unconfigured:** Node is created but empty.
2.  **Inactive:** Drivers are loaded, but Motors are DISABLED.
3.  **Active:** Main loop running, Motors ENABLED.
4.  **Finalized:** Cleanup and Shutdown.

### **Lab Procedure**
1.  **The Class:** Change your Controller to inherit from `rclpy.lifecycle.LifecycleNode`.
2.  **State Transitions:**
    *   `on_configure()`: Connect to Flight Controller (UART). Check battery. (Motors still OFF).
    *   `on_activate()`: Enable the PWM/DSHOT output. Start the Control Loop timer.
    *   `on_deactivate()`: **Safety Critical.** Immediately set all motors to 0. Stop the timer.
    *   `on_shutdown()`: Close UART connection.
3.  **The Test:**
    *   Run the node. It should be silent.
    *   Terminal: `ros2 lifecycle set /squid_controller configure` -> "Configured".
    *   Terminal: `ros2 lifecycle set /squid_controller activate` -> **Props Spin**.
    *   Terminal: `ros2 lifecycle set /squid_controller deactivate` -> **Props Stop**.

### **Deliverable**
*   A screen recording of you using the CLI to transition the node states and controlling the motor status.

---

## **Check: The Replica**
**Does it still fly?**

The goal is Feature Parity. The drone should fly *exactly* as well as it did in Module 5, but now the code is modular.

1.  **Test:** Repeat the "Hover Test" (Module 5).
2.  **Verify:** If the drone jitters, check your topic frequency (`ros2 topic hz /sensors/imu`). Latency in ROS 2 can be tricky!

**Submission:** A screenshot of Foxglove Studio displaying your flight data.

---
## **Theoretical Foundations**

### Lecture 6: Distributed Robotics Architecture & Middleware

#### **1. Middleware Topology & DDS (The Data Backbone)**
We transition from monolithic scripts to a distributed system using **Robot Operating System 2 (ROS 2)**, which sits on top of the **Data Distribution Service (DDS)** standard.
*   **The Participant Model:** Every Node is a "DDS Participant." It doesn't need to know the IP of other nodes; it simply declares its intent to "Publish" or "Subscribe" to a Global Data Space.
*   **The Discovery Problem:** On the Pi Zero, the CPU can hit $100\text{%}$ simply trying to "Discover" other nodes using the default Simple Discovery Protocol (SDP). We implement a **Discovery Server** (the laptop acts as the "Phonebook") to reduce network multicast traffic and preserve CPU cycles for flight.

#### **2. Quality of Service (QoS) & Information Theory**
Not all data requires the same delivery guarantees.
*   **Best Effort (UDP-like):** Used for IMU and Lidar streams. If a packet is lost, we drop it. The "Freshness" of the state is more important than the "History."
*   **Reliable (TCP-like):** Used for Mission Commands (e.g., "Land Now"). We utilize a **Deadline QoS** to ensure that if a message doesn't arrive within $100\text{ms}$, a hardware failsafe is triggered.

#### **3. Real-Time OS (RTOS) Priority & Preemption**
Linux is inherently "Fair"—it gives every process a turn. 
*   **The Danger:** If the Pi runs a system update, your PID thread will be paused.
*   **The Fix:** We utilize the **SCHED_FIFO** scheduler. We set the flight controller thread priority to `99`. This tells the Linux kernel: "This thread is more important than the Operating System itself. Never interrupt it."

**Next Step:** [Module 7: State Estimation](../Module_07_State_Estimation/Module_07_Lecture.md)

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"We are moving from a 'Single Brain' script to a 'Nervous System' of independent agents. ROS 2 is the diplomat that allows your Lidar driver and your PID controller to speak the same language. Today, we stop building a 'hobby project' and start building an 'industrial platform.' We are teaching our robots how to talk to each other without shouting."

### **Deep Research Context: Zero-Copy Transport**
In research fleets (e.g., swarms of 50 drones), memory management is the primary bottleneck. Normally, sending a message from Node A to Node B involves copying data in RAM. For high-resolution camera feeds, this is fatal. Mention **Shared Memory Transport** (e.g., Iceoryx). It allows the 'Publisher' to simply send a memory pointer to the 'Subscriber,' reducing data-transfer latency from milliseconds to microseconds.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Explain the benefit of "Decoupling" nodes using the Pub/Sub model.
- [ ] Identify the CPU bottleneck of the Simple Discovery Protocol on low-power hardware.
- [ ] Describe the difference between "Best Effort" and "Reliable" QoS settings.
- [ ] Implement a basic state machine using Lifecycle Node transitions.

---

## **Further Reading & Bibliography**

### **Frameworks**
*   **Macenski, S., et al. (2022).** *"Robot Operating System 2: Design, architecture, and uses in the wild."* Science Robotics. (The definitive academic overview).
*   **Object Management Group (OMG).** *Data Distribution Service (DDS) Specification v1.4.* (The industry standard).

### **Benchmarks**
*   **Maruyama, Y., Kato, S., & Azumi, T. (2016).** *"Exploring the performance of ROS2."* IEEE International Conference on Embedded and Real-Time Computing.

---

[Previous Module](../../Phase_3_The_Engineer/Module_05_Control_Theory/Module_05_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_06_5_Reliability/Module_06_5_Lecture.md)