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

---

## **The "Panic" Test**
**Safety Verification.**

1.  Put the drone in **AUTO** mode (Props Off).
2.  Simulate a Wi-Fi disconnect (unplug your router or turn off laptop Wi-Fi).
3.  **Observe:** Did the drone trigger the heartbeat failsafe within 500ms?
4.  Flick the RC switch to **MANUAL**.
5.  **Observe:** Did you regain control of the motors immediately?

**Submission:** A logic flow diagram showing your Failsafe State Machine.
