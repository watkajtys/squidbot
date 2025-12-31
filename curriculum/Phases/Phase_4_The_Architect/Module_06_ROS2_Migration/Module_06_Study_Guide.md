[Return to Course Map](../../../../COURSE_MAP.md)

# Study Guide 6: The ROS 2 Migration
**Module 6: The Architect (Scale and Safety)**

### Critical Takeaways
1.  **Distributed Computing:** ROS 2 enables the decomposition of the drone's intelligence into independent nodes (e.g., Sensing, Planning, Control). This modularity is a safety requirement; it ensures that a software crash in a high-level "Mapping" node does not affect the real-time execution of the low-level "Motor Mixer."
2.  **The Pub/Sub Model:** Data exchange is handled via a Topic-based Publish/Subscribe model. This decouples the data producer from the consumer. A "State Estimator" can publish the drone's position to `/odometry` without knowing if the listener is a Human Dashboard or a Swarm Coordinator.
3.  **DDS (Data Distribution Service):** The middleware of ROS 2. It handles peer-to-peer discovery and provides granular control over **Quality of Service (QoS)**. You can prioritize "Reliability" (wait for every packet) for a mission-critical command, and "Best Effort" (drop packets if needed) for high-bandwidth video streams.
4.  **Message Serialization:** ROS 2 automatically converts complex Python/C++ objects into a standard binary format (CDR). This ensures that a node written in Python can talk to a node written in C++ with zero overhead or "parsing" logic.

### Mental Models and Field Notes
*   **The Shout vs. The Whisper:** Publishing to a ROS topic is like shouting into a room. You don't care who hears you; you just report the news. Subscribing is like choosing which "radio station" to listen to. This is much more flexible than a direct function call (a "whisper").
*   **The Distributed Brain:** Think of your drone as a team of specialists. One specialist only knows how to read the IMU. One only knows how to do math. ROS 2 is the "meeting room" where they exchange notes. If the "math specialist" takes too long to think, the "IMU specialist" keeps working regardless.
*   **QoS (Priority):** In a crisis, you want to hear the "Fire Alarm" (Kill Command) even if the "Background Music" (Telemetry) is loud. QoS is how we tell the network which data is life-critical.

### Frontier Facts and Historical Context
*   **Space-ROS:** NASA and Blue Origin are currently developing "Space-ROS"—a flight-certified version of ROS 2 for use in satellites and lunar rovers. By learning ROS 2 on the Squid Drone, you are using the same architectural patterns that will likely control the next generation of deep-space robotics.
*   **The Origins of ROS:** ROS was originally created at Stanford University and Willow Garage in 2007. It was designed for a robot called the "PR2" which had two arms and was supposed to be able to fold laundry. While the PR2 was too expensive to catch on, its software "brain" became the standard for the entire world of robotics.
*   **DDS and Nuclear Subs:** The underlying technology of ROS 2 (DDS) was originally developed for use in mission-critical systems like US Navy nuclear submarines and air traffic control centers. It is designed to be "Always On" and self-healing.

---

### The Squid Games: Level 6
**The Latency Hunt Challenge**
Measure the "Transport Latency" of your ROS 2 stack. Write a script that publishes a timestamp from one node and reads it in another.
*   **The Goal:** Calculate the average time (in milliseconds) it takes for a message to travel from the "IMU Node" to the "Controller Node."
*   **Win Condition:** Average latency must be < 5ms. If it is higher, you must optimize your **DDS Tuning** parameters (e.g., switching from Wi-Fi to Localhost-only transport).

---

### Module 6 Quiz
1.  **DDS:** What is the difference between the "Reliable" and "Best Effort" QoS policies? Which one would you use for a `/emergency_stop` topic?
2.  **Namespaces:** How do you run two identical Squid Drones in the same room without their ROS topics clashing?
3.  **Middleware:** What is the purpose of the `rmw_implementation`?
4.  **Serialization:** Why is binary CDR serialization faster than sending data as JSON strings?

---
*Reference: Module 6 in docs/Module_Lecture.md*
