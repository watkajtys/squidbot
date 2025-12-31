# The Robotics Debugging Guide
**"Why is it doing that?"**

In robotics, "It doesn't work" could mean 50 different things. Use this guide to narrow it down.

---

## **1. The "Is it Plugged In?" Phase (Hardware)**
Before you change your code, check the copper.
*   **The "Wiggle" Test:** While the drone is on the bench (Props OFF), gently wiggle your UART and I2C wires. If your telemetry dashboard flickers or drops, your solder joint is "Cold" or your connector is loose.
*   **Voltage Sag:** If the drone flies for 1 minute then flips, check your battery voltage logs. If the voltage "dips" significantly when you punch the throttle, your battery C-rating is too low or your BEC is struggling.

---

## **2. The "Is it Thinking?" Phase (Software)**
*   **`top` / `htop`:** Run this on the Pi via SSH. If `python3` is using > 80% CPU, your loop will stutter, your math will be wrong, and the drone will oscillate.
*   **`ros2 topic hz /topic_name`:** Is your data actually arriving at the speed you think? If your Lidar is supposed to be 30Hz but is only 5Hz, your EKF will fail.
*   **The Print Statement is your Friend (But also your Enemy):**
    *   *Do:* Print sensor values to see if they are "sane" (e.g., Altitude shouldn't be -500m).
    *   *Don't:* Print inside a 1000Hz loop. Printing to a terminal is **slow**. It will add "Jitter" and crash your drone.

---

## **3. The "Is the Math Mathing?" Phase (Physics)**
*   **Sign Errors:** The #1 cause of crashes.
    *   *The Test:* Tilt the drone right. Does the "Roll" value go UP or DOWN?
    *   *The Danger:* If your code expects UP but gets DOWN, the PID will "correct" by tilting even further right. **Instant Flip.**
*   **Unit Mismatch:** Are you mixing Radians and Degrees? Meters and Centimeters?
    *   *Rule:* Convert everything to **SI Units** (Meters, Radians, Seconds) the moment it leaves the driver.

---

## **4. The "Golden Rule" of Debugging**
**Change ONLY ONE thing at a time.**
If you change your PID gains, your filter frequency, and your motor PWM at the same time, you will never know which one fixed (or broke) the flight.
