# The "Squid" Course: Fun-First Progression
**Platform:** Pavo20 / Pi Zero 2 W / Python / WebSockets
**Philosophy:** "Play first, Optimize later."

---

## **PART I: THE MAKER (Hardware & "Hello World")**
*Goal: Get the machine built and moving using simple, readable scripts.*

### **Module 0: The Build Party**
*   **0.1:** Anatomy: What is an ESC? What is a Flight Controller?
*   **0.2:** Soldering 101: The Art of the "Shiny Joint" (Building the Power Train).
*   **0.3:** The "Smoke Test": Plugging in the battery without exploding.
*   **0.4:** The Brain Transplant: Setting up Linux on the Pi (Headless).

### **Module 1: The Puppet Master (Manual Control)**
*   **1.1:** Motor Twitch: Sending your first MSP command to spin Prop #1.
*   **1.2:** The "Keyboard Pilot": Writing a script to fly the drone like a video game (WASD keys) over Wi-Fi.
    *   *Fun Factor:* You are flying your drone! (Carefully).
*   **1.3:** The Kill Switch: Coding the "Spacebar = DROP" safety feature.

---

## **PART II: THE WEB DEVELOPER (Telemetry & Dashboards)**
*Goal: Use your Web Dev skills to visualize what the drone is "feeling."*

### **Module 2: The Full Stack Drone**
*   **2.1:** The Backend: Writing a Python **WebSocket Server** on the Pi.
*   **2.2:** The API: Streaming Sensor Data (Distance, Battery Voltage, Gyro) as JSON.
*   **2.3:** The Frontend: Building a simple HTML/JS Dashboard on your Laptop to graph the altitude in real-time.
    *   *Fun Factor:* "Look at this! I built a flight computer in my browser!"

### **Module 3: The Camera (FPV)**
*   **3.1:** MJPEG Streaming: Creating a low-latency video feed.
*   **3.2:** The Overlay: Drawing "Iron Man" style HUD data (Battery, Height) over the video feed on your dashboard.

---

## **PART III: THE SCIENTIST (Autonomy Logic)**
*Goal: The drone stops listening to your keyboard and starts thinking.*

### **Module 4: The Reflexes (Simple Logic)**
*   **4.1:** "The Wall": If `ToF < 50cm`, stop moving forward.
*   **4.2:** "The Floor is Lava": If `Altitude < 50cm`, throttle up.
*   **4.3:** "The Security Guard": Sit on the desk. If the Camera detects motion, arm the motors (Roar).

### **Module 5: The Smooth Operator (PID Control)**
*   **5.1:** The Theory: Why `if/else` makes the drone wobble.
*   **5.2:** The Code: Writing a PID class in Python.
*   **5.3:** Tuning: Adjusting the numbers until it hovers like a statue.

---

## **PART IV: THE ENGINEER (Professional Refactoring)**
*Goal: We hit the limits of Python/JSON. Now we need ROS2.*

### **Module 6: The Upgrade (ROS2)**
*   **6.1:** Why Python is too slow: Measuring Jitter.
*   **6.2:** Porting the "Backend" to ROS2 Nodes.
*   **6.3:** Replacing WebSockets with DDS (Industry Standard).

### **Module 7: The Math (State Estimation)**
*   **7.1:** Sensor Fusion: Combining Flow + IMU.
*   **7.2:** The Kalman Filter: Fixing the drift.

---

## **PART V: THE ACE (Advanced Combat)**
*Goal: The original "Hunter-Killer" goals.*

### **Module 8: Mapping & Perception**
*   **8.1:** 3D Mapping the room.
*   **8.2:** Finding targets.

### **Module 9: The Dogfight**
*   **9.1:** Interception Logic.
*   **9.2:** Zero-G Maneuvers.

---

## **Graduation Checklist**
1.  **Phase 1:** Fly via Keyboard.
2.  **Phase 2:** View Telemetry in Browser.
3.  **Phase 3:** Autonomous Hover.
4.  **Phase 4:** Port to ROS2.
5.  **Phase 5:** Autonomous Hunt.