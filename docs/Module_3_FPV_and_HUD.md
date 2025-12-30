# Module 3: FPV & HUD
**"Flying by wire. Seeing by numbers."**

In Module 2, you built a data dashboard. In Module 3, we add the most bandwidth-heavy sensor of all: Vision. You will build a low-latency video streaming pipeline and overlay your Lidar data onto it, creating an Augmented Reality (AR) Heads-Up Display (HUD).

---

## **3.1 Video Pipeline: Hardware Encoding**

### **Objective**
Stream 720p video from the drone to the laptop with <200ms latency.

### **Theory**
*   **MJPEG:** Sending JPEG images one by one. Simple, but high bandwidth.
*   **H.264:** Compresses data by only sending *changes* between frames. Harder to implement, but much faster.
*   **The Hardware:** The Pi Zero 2 W has a dedicated hardware encoder (`OpenMAX` or `V4L2 M2M`). If you use software encoding (CPU), the Pi will melt and the stream will lag.

### **Lab Procedure**
1.  **The Source:** Use `libcamera-vid` or GStreamer to capture video from the Arducam.
2.  **The Pipe:**
    *   **On Drone:** `libcamera-vid -t 0 --inline -o - | nc -l -p 5000` (Simple Netcat tunnel for testing).
    *   **Advanced:** Use a GStreamer pipeline with RTP/UDP.
    *   `gst-launch-1.0 libcamerasrc ! video/x-raw,width=640,height=480,framerate=30/1 ! videoconvert ! x264enc tune=zerolatency ! rtph264pay ! udpsink host=LAPTOP_IP port=5600`
3.  **The Sink (Laptop):**
    *   Receive the stream using GStreamer or VLC to verify.

### **Deliverable**
*   A screenshot of your laptop receiving the live video feed.
*   Measure the latency (Film a stopwatch on your laptop screen with the drone; subtract the time on the feed from the time on the screen).

---

## **3.2 Augmented Reality (The HUD)**

### **Objective**
Overlay the Lidar distance (Module 1) and Telemetry (Module 2) onto the video feed.

### **Theory**
We do not want to draw on the video *on the drone* (burns CPU). We send the video and the data separately and merge them *on the laptop*.

### **Lab Procedure**
1.  **Frontend:** Update your React/HTML dashboard from Module 2.
2.  **The Canvas:**
    *   Layer 1 (Bottom): The Video Feed (using an `<img>` tag for MJPEG or a WebRTC player).
    *   Layer 2 (Top): An HTML5 `<canvas>` transparent overlay.
3.  **The Render:**
    *   Use the Telemetry WebSocket data to draw a "Artificial Horizon" line or a simple text box: `ALTITUDE: 0.4m`.
    *   Draw a "Reticle" in the center.

### **Deliverable**
*   A working web page where you see the camera view, and when you move your hand over the Lidar, the number on the screen updates in sync with the video.

---

## **3.3 System Identification: Latency**

### **Objective**
Measure the glass-to-glass and glass-to-motor latency.

### **Lab Procedure**
1.  **Photon-to-Photon:** Film your laptop screen showing the drone's video feed *with* the drone's camera. Use a 240fps phone camera. Measure the frame difference between an event (e.g., LED flash) happening in reality vs. on screen.
2.  **Photon-to-Motion:** Flash an LED in front of the drone. Measure the time until the motors twitch (using the "Reflex" logic from Module 1). This characterizes the total processing delay $T_{pipeline}$.

---

## **Check: Instrument Flight**
**The Ultimate Test of Trust.**

1.  **The Setup:** Place the drone in another room (or cover your eyes).
2.  **The Task:**
    *   Have a partner place a cardboard box somewhere in the room.
    *   Using *only* your Dashboard (Video + Lidar HUD), guide the drone (carry it by hand if not flying yet, or fly if stable) to within 50cm of the box.
    *   Use the Lidar reading on your HUD to confirm the distance.
3.  **Verification:** Your partner confirms you are actually 50cm away.

**Submission:** A short screen recording of your HUD during the approach.
