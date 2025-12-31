[Previous Module](../Module_06_ROS2_Migration/Module_06_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_07_State_Estimation/Module_07_Lecture.md)

---

# Module 6.5: Heartbeat & The "Human-in-the-Loop"
**"Who is driving the bus?"**

In professional robotics, autonomous systems must always have a "Safety Handoff." If the Pi crashes, or the Wi-Fi drops, the Human Pilot must be able to take over instantly. This is the difference between a minor inconvenience and a catastrophic crash.

---

## **6.5.1 The Heartbeat Protocol**

### **Objective**
Implement a "Dead Man's Switch" between your Laptop (GCS) and the Drone.

### **Theory**
A **Heartbeat** is a small packet sent at a regular interval (e.g., 10Hz).
*   **If the Heartbeat is present:** The drone knows the GCS is alive and it's safe to continue autonomous flight.
*   **If the Heartbeat stops:** (Network loss, Laptop battery dies, software crash), the drone must execute a **Failsafe**.

### **Lab Procedure**
1.  **Laptop Side (`gcs_heartbeat.py`):**
    *   Publish a `std_msgs/Header` to `/heartbeat/gcs` at 10Hz.
2.  **Drone Side (`failsafe_monitor.py`):**
    *   Subscribe to `/heartbeat/gcs`.
    *   Use a Timer to check the `last_received_time`.
    *   **The Logic:**
        ```python
        if current_time - last_received_time > 0.5: # 500ms timeout
            trigger_failsafe()
        ```

---

## **6.5.2 The Handoff (RC Override)**

### **Objective**
Allow the RC Controller to "steal" control from the Pi.

### **Theory**
The Flight Controller (FC) receives signals from your RC Radio (TX). We will configure the FC to treat the Pi as an "External Input."
*   **Mode Switch:** Use a 3-position switch on your RC Radio.
    1.  **Position 1 (MANUAL):** RC Radio controls the drone directly. Pi input is ignored.
    2.  **Position 2 (STABILIZE):** FC handles leveling, but Pilot handles throttle.
    3.  **Position 3 (AUTO):** The Pi Zero has full control.

### **Lab Procedure**
1.  **Betaflight Setup:**
    *   Go to the "Modes" tab.
    *   Assign an AUX channel to a "Pi_Override" or "MSP_Override" mode (varies by setup).
2.  **The Mixer Node Refactor:**
    *   Modify your `mixer_node.py` to listen to the RC receiver status.
    *   **The Logic:**
        ```python
        if rc_switch_is_manual:
            pass_through_rc_commands()
        else:
            apply_pi_autonomous_commands()
        ```

---

## **6.5.3 Graceful Degradation: The "Blind" Landing**

### **Objective**
Survive a sensor failure.

### **Theory**
What if the Lidar (Altitude) fails mid-flight? 
*   **Bad Logic:** Panic and shut off motors. (Drone falls).
*   **Good Logic:** Switch to "Open Loop" descent. Slowly reduce throttle over 3 seconds to attempt a soft landing.

### **Lab Procedure**
1.  **Induce Failure:** While the drone is in "Virtual Hover" (Props Off), unplug the Lidar.
2.  **The Watchdog:** Implement a check in your state estimator.
3.  **The Response:** The drone should enter `STATE_EMERGENCY_LANDING`, ignore the target altitude, and slowly ramp down `motor_mixer(thrust_ramp_down, 0, 0, 0)`.

### **6.5.4 Just-In-Time Math: The Sum of the Parts (Pre-integration)**
**"Compressing Time"**

Your IMU runs at 1000Hz. Your Graph Optimizer (Module 7) runs at 10Hz.
*   **The Problem:** You have 100 IMU measurements for every 1 Optimization step. Adding 100 nodes to the graph is too slow.
*   **The Solution:** We "Pre-integrate" them. We sum up the 100 small moves into one "Big Move" (Delta Position, Delta Velocity, Delta Rotation).
*   **The Benefit:** The Optimizer treats the 100 steps as a single constraint ("The drone moved 5cm North with 99% confidence"). This makes the math 100x faster.

**AI Prompt:** "Explain the concept of IMU Pre-integration Factors in GTSAM. How do we condense multiple accelerometer readings into a single relative pose change?"

---

## **The "Panic" Test**
**Safety Verification.**

1.  Put the drone in **AUTO** mode (Props Off).
2.  Simulate a Wi-Fi disconnect (unplug your router or turn off laptop Wi-Fi).
3.  **Observe:** Did the drone trigger the heartbeat failsafe within 500ms?
4.  Flick the RC switch to **MANUAL**.
5.  **Observe:** Did you regain control of the motors immediately?

**Submission:** A logic flow diagram showing your Failsafe State Machine.

---
## **Theoretical Foundations**

### Lecture 6.5: Fault-Tolerant Systems & Reliability

#### **1. The Byzantine Generals Problem**
How do you trust a sensor when it could be lying or broken? In professional robotics, we use **Redundancy**. If we have two Lidars and one says 10m while the other says 0.5m, we look at the IMU to "break the tie." This is the core of **Fault-Tolerant Control**.

#### **2. Watchdog Architecture**
A Watchdog is a specialized circuit or thread that resets the system if it isn't "petted" regularly.
*   **The Hardware Watchdog:** On the Pi Zero, we use the `/dev/watchdog` device. If the Python process hangs, the kernel itself will reboot the Pi to prevent a runaway drone.

#### **3. Formal Methods: Safety Barriers**
We can mathematically prove that a drone will never hit a wall using **Control Barrier Functions (CBFs)**. We define a "Safe Set" of states and ensure the control law always "pushes" the drone back into that set before a crash becomes inevitable.

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"In robotics, 'Success' is boring. 'Failure' is where the research happens. Every commercial drone you buy has thousands of hours of failsafe logic that you never see until something goes wrong. Today, you are the safety engineer. You are the one who ensures that when the Wi-Fi dies, the drone doesn't become a projectile."

### **Deep Research Context**
*   **The Livelock:** Explain that a program can be 'running' but still 'frozen' (e.g., stuck in an infinite loop). A simple heartbeat that just says 'I am alive' isn't enough. A research-level heartbeat should include a **Sequence Number** and a **Status Hash** to prove the code is actually progressing through its logic.
*   **RC Latency:** Professional pilots can detect 20ms of lag. When you 'Hand off' from Auto to Manual, the transition must be bumpless. This means the RC commands should be 'shadowing' the Auto commands so there isn't a sudden jump in throttle.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Explain the physical danger of "Head-of-Line Blocking" in safety-critical systems.
- [ ] Diagram a Heartbeat/Watchdog architecture for a multi-processor robot.
- [ ] Describe the "Bumpless" handoff requirement for RC-Override modes.
- [ ] Define a "Safe Set" for a Control Barrier Function.

---

## **Further Reading & Bibliography**

### **Dependability**
*   **Knight, J. C. (2002).** *"Fundamentals of Dependable Computing."* IEEE Control Systems Magazine. (Foundations of safety-critical software).
*   **Koopman, P. (2010).** *Better Embedded System Software.* Dr. Phil's Press. (Practical guide to robust architecture).

### **Formal Methods**
*   **Ames, A. D., et al. (2019).** *"Control Barrier Functions: Theory and Applications."* IEEE European Control Conference.

---

[Previous Module](../Module_06_ROS2_Migration/Module_06_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_07_State_Estimation/Module_07_Lecture.md)