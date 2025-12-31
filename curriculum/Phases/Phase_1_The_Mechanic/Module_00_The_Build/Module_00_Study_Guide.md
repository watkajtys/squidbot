[Return to Course Map](../../../../COURSE_MAP.md)

# Study Guide 0: Systems Engineering and The Build
**Module 0: The Mechanic (Hardware and Instincts)**

### Critical Takeaways
1.  **The Real-Time Law:** In robotics, determinism is more critical than raw speed. A steady 50Hz control loop that never misses a deadline is fundamentally safer than an erratic 200Hz loop. High-speed oscillations in the loop timing (jitter) can cause the drone to physically resonate, leading to structural failure.
2.  **The Ghost of the Past:** Sensors do not report the present; they report the world as it was during the last sample. Latency is the "ghost" your code must constantly account for. By the time your algorithm processes a frame, the drone has already moved several centimeters.
3.  **Hardware Hygiene:** The Battery Eliminator Circuit (BEC) is the gatekeeper of your electronics. A single voltage spike or a short circuit at the power rail level can instantly destroy the Raspberry Pi's SOC. Always verify a 5.0V to 5.2V output under load before connecting the "brain."

### Mental Models and Field Notes
*   **The Shadow of the Past:** When you read a sensor, you are effectively looking at a "replay" of history. All advanced control is the art of predicting the future to compensate for this shadow. If your control loop takes 20ms to process, you are essentially flying a drone that is 20ms "in the past" at all times.
*   **The Mars Pathfinder Lesson:** In 1997, the Mars Pathfinder kept resetting because a low-priority task held a resource needed by a high-priority task (Priority Inversion). In the field, we don't just write code; we manage "Time Budgets." If you overspend your time on a low-priority logging task, your high-priority flight task will go "bankrupt" and the drone will fall.
*   **Nyquist-Shannon in the Real World:** It is not enough to sample "fast." You must sample at least twice as fast as the highest frequency present in the system. If your motors vibrate at 120Hz and you sample at 100Hz, those vibrations will "fold" into your data as a slow, 20Hz phantom wave that your PID controller will try (and fail) to correct.

### Frontier Facts and Historical Context
*   **The "Billion-Node" Swarm:** Research in "Asynchronous Systems" is moving away from the 50Hz global clock. Some cutting-edge swarms use "Event-Based" sensors (Neuromorphic) that only send data when something changes.
*   **Neuromorphic Computing:** Standard AI uses a "Global Clock." Neuromorphic systems mirror how biological systems, like the eyes of a fly, process motion—using microsecond reaction times while consuming 1/100th of the power of a standard Raspberry Pi.
*   **Space-ROS:** NASA and Blue Origin are currently developing "Space-ROS"—a flight-certified version of ROS 2 for use in satellites and lunar rovers. By learning these foundations, you are using the same patterns that control the next generation of deep-space robotics.

---

### The Squid Games: Level 0
**The Heartbeat Challenge**
Write a script that takes a string (e.g., "SQUID") and blinks the onboard LED in perfect Morse Code. 
*   **The Goal:** Observe the LED with a high-speed camera or an oscilloscope. Are the "dots" exactly 100ms? Are the "dashes" exactly 300ms?
*   **Win Condition:** Seeing the hardware respond to your logic with sub-millisecond precision. If the timing drifts, your real-time scheduling is not deterministic.

---

### Module 0 Quiz
1.  **Aliasing:** You are measuring a motor vibration oscillating at 120Hz. If your sensor samples at 100Hz, what "phantom" frequency will appear in your data?
2.  **Scheduling:** Explain the difference between Hard and Soft Real-Time systems. Which classification does the Squid Drone flight controller fall under?
3.  **Numerical Integration:** Why is the "Forward Euler" method dangerous for long-duration simulations compared to a "Runge-Kutta 4" or "Symplectic" integrator?
4.  **Power Electronics:** Why is it catastrophic to connect a 3S LiPo battery directly to the Raspberry Pi GPIO pins?

---
*Reference: Lecture 0 (Systems Engineering) in docs/LECTURES.md*
