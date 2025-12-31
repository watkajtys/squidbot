# Study Guide 6.5: Heartbeat and The Human-in-the-Loop
**Module 6.5: The Architect (Failsafes and Reliability)**

### Critical Takeaways
1.  **The Heartbeat is the Life-Line:** A robotic system without a heartbeat is a "Runaway" waiting to happen. In distributed systems (like ROS 2 over Wi-Fi), connectivity is a variable, not a constant. You must design for the "Total Loss of Signal" (TLS) as a standard operating state, not an exception.
2.  **Determinism vs. Connectivity:** Your drone's control loop must remain deterministic even when the network is erratic. Failsafe logic should reside on the "Edge" (the drone itself), not the ground station. If the ground station dies, the drone's onboard "Ego-Heartbeat" should take over.
3.  **Bumpless Handoff:** Transitioning from autonomous control to manual RC control is the most dangerous moment in flight. If the RC sticks are at 0% throttle but the Pi was at 60% during the handoff, the drone will drop like a stone. A robust system "Shadows" the autonomous commands so the pilot is always in sync.

### Mental Models and Field Notes
*   **The Dead Man's Switch:** Originating from locomotive engineering, this model ensures that if the operator becomes incapacitated, the system enters a safe state. In drones, "incapacitation" usually means a software crash or network jamming.
*   **Graceful Degradation:** A system should not simply "Fail" (Binary). It should "Degrade" (Spectrum). If you lose GPS, you degrade to Optical Flow. If you lose Optical Flow, you degrade to IMU-only leveling. If you lose the IMU, you perform a "Motor Cut."
*   **The "Livelock" Paradox:** Your code can be "Running" (the CPU is busy) but "Frozen" (it's stuck in an infinite loop or waiting for a socket that will never open). A simple heartbeat that toggles a bit is insufficient; you must send a "Status Hash" that proves the main flight loop is actually iterating.

### Frontier Facts and Historical Context
*   **The Apollo 11 "1202" Error:** During the moon landing, the guidance computer was overwhelmed with data (a "Priority Inversion" of sorts). Because of robust scheduling, it dropped low-priority tasks and kept the landing logic alive. This is the ultimate example of "Reliability under Pressure."
*   **Formal Methods (CBFs):** In 2025+, we no longer just "Hope" the drone doesn't hit a wall. We use **Control Barrier Functions**. These are mathematical "Invisible Walls" that override the user's input only when a collision is mathematically certain.
*   **Erlang & Nine Nines:** Some aerospace companies are looking at the Erlang/Elixir "Let it Crash" philosophy for non-critical telemetry nodes. If a node fails, the supervisor simply restarts it in milliseconds, rather than trying to handle every possible edge case.

---

### The Squid Games: Level 6.5
**The Silence Challenge**
Fly the drone in a stabilized hover (Props Off for bench test). While it is hovering, manually disable your laptop's Wi-Fi adapter.
*   **The Goal:** The drone must detect the loss of the GCS heartbeat and enter `STATE_EMERGENCY_LAND` within 500ms.
*   **Win Condition:** Seeing the motor outputs ramp down to zero in a controlled sequence rather than just stopping or continuing to spin at hover speed.

---

### Module 6.5 Quiz
1.  **Timeouts:** Why is a 500ms heartbeat timeout chosen over a 50ms timeout for a drone flying over Wi-Fi? (Hint: Consider "Bursty" network traffic).
2.  **Redundancy:** Describe the difference between "Active" and "Passive" redundancy in sensor systems.
3.  **Byzantine Generals:** You have 3 IMUs. Two say the drone is tilted left, one says it is tilted right. How do you decide which to trust?
4.  **Handoffs:** Explain the concept of a "Bumpless" transition and why it requires the RC transmitter to receive telemetry from the drone.

---
*Reference: Lecture 6.5 (Heartbeat) in curriculum/Phases/Phase_4_The_Architect/Module_06_5_Reliability/Module_06_5_Lecture.md*
