# Study Guide 2: The Telemetry Stack
**Module 2: Observability and Data Flow**

### Critical Takeaways
1.  **Observability Theory:** In control systems, a system is "observable" if its internal state can be fully determined by its external outputs. A telemetry stack is the physical implementation of this theory. Without it, the drone's state is a "Black Box," making root-cause analysis of flight failures impossible.
2.  **Bandwidth vs. Latency Tradeoff:** You cannot send everything. Flooding a 115200 baud serial link with 1kHz IMU data will cause a "Buffer Bloat" or packet loss, resulting in delayed control commands. You must prioritize high-frequency "State" data (Attitude, Altitude) over low-frequency "Health" data (Battery, Temperature).
3.  **The MAVLink Standard:** Most professional drones use the MAVLink protocol. It is a binary, packet-oriented protocol that includes CRC checksums and sequence numbers to ensure that data is not only correct but arrived in the right order.
4.  **On-Board vs. Off-Board Logging:** Real-time telemetry is for the human pilot; high-speed Blackbox logging (to SD card) is for the engineer. Blackbox logs capture data at the full rate of the PID loop (up to 8kHz), allowing for sub-millisecond vibration analysis.

### Mental Models and Field Notes
*   **The Pulse of the Machine:** Think of telemetry as the drone's "nervous system" extending to your laptop. If the pulse is steady, the drone is healthy. If the telemetry "freezes" for even half a second, the drone is effectively "blind" to your commands.
*   **Packet Shredding:** In a wireless environment, imagine your data packets are being thrown through a shredder. Only the packets that are perfectly reassembled by the CRC are "real." Never write code that assumes every packet you send will be received.
*   **Information Density:** Every byte you send costs energy and time. Sending a floating-point number as a string (e.g., "12.345") takes 6 bytes. Sending it as a 32-bit IEEE 754 float takes only 4 bytes. In high-speed robotics, we always choose the binary representation.

### Frontier Facts and Historical Context
*   **Telemetry-over-Cellular (5G):** Standard drone telemetry is limited by line-of-sight. However, researchers are now using 5G "Slicing" to send telemetry through the cellular network. This allows a drone in one city to be monitored and controlled from another with sub-30ms latency.
*   **MAVLink History:** MAVLink (Micro Air Vehicle Link) was created in 2009 by Lorenz Meier. Before it existed, every drone manufacturer had their own "secret" language, making it impossible for different robots to talk to each other. MAVLink created the "Esperanto" of the drone world.
*   **The Black Box:** The term "Black Box" comes from early aviation where flight recorders were literally painted black to keep the film inside from being exposed to light. Today, your drone's "Black Box" is just a high-speed `.csv` file on a microSD card, but it is just as critical for survival.

---

### The Squid Games: Level 2
**The Virtual Level Challenge**
Using your telemetry dashboard from Module 3, tilt the drone in your hand. The "Virtual Horizon" must stay perfectly level with the actual floor, mirroring the drone's orientation in real-time.
*   **The Goal:** Optimize your serial buffer logic until the visual lag is imperceptible to the human eye (< 50ms).
*   **Win Condition:** A video demonstration of a "Lag-Free" horizon. If the horizon "stutters" or "drifts," your telemetry parser is likely blocking the main thread.

---

### Module 2 Quiz
1.  **Serial Communication:** Calculate the theoretical maximum number of 32-byte packets you can send per second over a 115200 baud UART link.
2.  **Buffering:** What is "FIFO" (First-In-First-Out), and why is it used for telemetry data? What happens if the "Producer" is faster than the "Consumer"?
3.  **MAVLink:** Why does MAVLink include a "System ID" and "Component ID" in every packet?
4.  **Debugging:** According to the `Robotics_Debugging_Guide.md`, what is the "Wiggle Test," and what hardware failure does it diagnose?

---
*Reference: Lecture 2 (Stochastic Processes) in docs/LECTURES.md*