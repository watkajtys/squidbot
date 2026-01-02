[Return to Course Map](../../../../COURSE_MAP.md)

# Study Guide 1: The Bare Metal API
**Module 1: Low-Level Drivers and Motor Mixing**

### Critical Takeaways
1.  **The Mixer Matrix:** A drone does not inherently understand "Move Left" or "Go Up." The Mixer Matrix is the mathematical translation layer that converts abstract control inputs (Throttle, Roll, Pitch, Yaw) into 4 specific PWM duties for the motors.
2.  **Information Integrity:** Communication via I2C or UART is subject to electromagnetic interference (EMI) from the motors. Every packet must be validated via Checksums (XOR) or Cyclic Redundancy Checks (CRC) to ensure a single bit-flip doesn't command 100% thrust.
3.  **The Watchdog Principle:** Always implement a safety timeout. If the high-level controller (the Pi) fails to send a heartbeat within 100ms, the low-level API (the Flight Controller) must automatically disarm the motors to prevent a flyaway.
4.  **Non-Blocking I/O (Async Logging):** Never write to the disk (SD card) inside the main flight loop. Disk I/O is stochastic and can cause 50ms+ hangs, leading to a loss of control authority. Use a threaded queue for all logging.
5.  **Numerical Representation:** Be vigilant regarding Endianness. Reading a 16-bit sensor value in the wrong byte order (Big-Endian vs Little-Endian) will result in values that are thousands of units off, causing immediate and violent instability.

### Mental Models and Field Notes
*   **Digital Trust:** In a high-vibration environment, every bit is "guilty until proven innocent." If you don't have a checksum, you don't have a communication link—you have a gamble.
*   **Numerical Decay:** Every floating-point operation loses a microscopic sliver of truth. Over millions of iterations in a flight loop, this "decay" can accumulate (Catastrophic Cancellation) until the drone's estimated state is physically impossible.
*   **The Anatomy of a Packet:** Think of a packet like a physical envelope. The "Header" is the address, the "Payload" is the letter, and the "Checksum" is the wax seal. If the seal is broken, you throw the whole letter away—you never try to "guess" what it said.

### Frontier Facts and Historical Context
*   **The "Limp Home" Mixer:** In `lab_1_1_mixer_matrix.py`, we explore how a 3-motor drone can still maintain level hover by re-calculating the Mixer Matrix in real-time. This "fault-tolerant" logic is what allows damaged drones to land safely rather than falling like a stone.
*   **Software-Defined Motors:** Traditional motors use a simple PWM signal. However, new "DShot" protocols allow the Flight Controller to talk to the motors digitally. This means the motor can talk back, telling the drone its current RPM, temperature, and even if it has a broken propeller.
*   **Endianness:** The terms "Big-Endian" and "Little-Endian" actually come from Gulliver's Travels, where two nations go to war over which end of a hard-boiled egg should be broken first. In robotics, choosing the wrong "end" of a 16-bit number is just as consequential.

---

### The Squid Games: Level 1
**The Morse Mixer Challenge**
Simulate a "Motor Failure" in your mixing code. Write a logic gate that can still maintain a level hover using only 3 motors by re-calculating the Mixer Matrix in real-time.
*   **Win Condition:** Successfully running `lab_1_1_mixer_matrix.py` with one motor disabled and seeing the other three compensate for the lost torque and thrust.

---

### Module 1 Quiz
1.  **Motor Mixing:** On a standard X-frame quadrotor, which motors must increase in RPM to execute a "Pitch Forward" maneuver?
2.  **Control Saturation:** If your throttle is at 100%, what happens to your ability to control Roll and Pitch? Explain why the mixer matrix must "clamp" its output.
3.  **Check-Summing:** Given the packet `[0xAA, 0x01, 0x05]`, show the step-by-step XOR calculation for a 1-byte checksum.
4.  **IEEE 754:** What is "Machine Epsilon," and why does it matter for long-duration flight code?

---
*Reference: Lecture 1 (Embedded Communication) in docs/LECTURES.md*