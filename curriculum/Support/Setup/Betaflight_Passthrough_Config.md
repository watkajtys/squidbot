# The Actuator Server: Configuring Betaflight for Python Control
**"One Drone. One Captain."**

By default, Betaflight wants to fly the drone. We need to disable its brain so it only acts as a "Slave" to our Raspberry Pi.

---

## **1. The PID Kill-Switch**
If Betaflight's PIDs are active, it will try to stabilize the drone. When your Python code also tries to stabilize, they will fight (Resonance).

**Action:** In Betaflight CLI, run:
```bash
# Disable internal PID stabilization
set p_pitch = 0
set i_pitch = 0
set d_pitch = 0
set p_roll = 0
set i_roll = 0
set d_roll = 0
save
```
*Note: We keep 'Yaw' and 'Throttle' PIDs active for now until we write our own in Module 5.*

---

## **2. MSP Override**
We must tell Betaflight to ignore the Radio Receiver (if any) and listen to the UART port.

**Action:** 
1.  Go to the **Configuration** tab.
2.  Set **Receiver Mode** to "Serial-based receiver".
3.  Set **Serial Receiver Provider** to "MSP".
4.  This allows your Python `send_motor_command()` to actually spin the motors.

---

## **3. The Emergency Failsafe**
**Crucial:** If your Python code hangs (`Ctrl+C` or a crash), the motors will keep spinning at the last commanded speed.

**Action:** 
1.  Go to the **Failsafe** tab.
2.  Set **Stage 2 - Settings** to "Drop" or "Land".
3.  Set **Guard Time** to 5 (0.5 seconds).
4.  **Result:** If the Pi stops sending MSP packets for 0.5s, Betaflight will kill the motors automatically. **THIS SAVES LIVES.**
--- [Return to Course Map](../../../COURSE_MAP.md)