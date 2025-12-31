[Previous Module](../Module_02_Telemetry_Stack/Module_02_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../../Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Lecture.md)

---

# Module 3: FPV & HUD (Augmented Reality)
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

### **3.2.1 Sub-Lab: The Virtual Horizon**
**"Seeing the Unseen Physics."**

A commercial pilot has a "Virtual Horizon" (PFD) that stays level with the earth even if the plane is upside down. You will build one.

1.  **Theory:** To draw a level line on a tilting video feed, you must rotate the line by the **Negative Roll** of the drone and shift it by the **Pitch**.
2.  **Lab Procedure:**
    *   In your HTML5 Canvas code, take the `roll` value (in radians) from the Telemetry stream.
    *   Use `ctx.rotate(-roll)` to counter-rotate the canvas.
    *   Draw a 200px horizontal line.
3.  **The Test:** Pick up the drone and tilt it left/right.
    *   **Success:** The line on your screen stays perfectly level with your desk/horizon, regardless of how you hold the drone.
4.  **Knowledge Step-up:** This is your first **Coordinate Transform**. You are transforming a line from "Inertial Space" (Earth) to "Image Space" (Camera).

### **3.2.2 Just-In-Time Math: The Pinhole Camera**
**"Why does the HUD float?"**

In the "Virtual Horizon" lab, you are mapping 3D space to a 2D screen.
*   **The Analogy:** Imagine looking through a window (the screen) with one eye closed (the camera).
    *   If you move your head back (change Focal Length), the tree outside looks smaller.
    *   If you tilt your head (Rotation), the horizon tilts.
*   **The Matrix ($K$):** The "Intrinsic Matrix" is just a mathematical description of where your eye is relative to the window.
    *   $f_x, f_y$: How close your eye is to the glass (Zoom).
    *   $c_x, c_y$: The center point of the glass.
*   **The Equation:** `Pixel_u = (f_x * X / Z) + c_x`. It's just similar triangles!

**AI Prompt:** "I have a 3D point [X, Y, Z] and a camera intrinsic matrix K. Explain how to project this point onto the 2D image plane [u, v] using homogeneous coordinates."

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

---
## **Theoretical Foundations**

### Lecture 3: Computer Vision & Projective Geometry

#### **1. The Pinhole Camera Model & Perspective Projection**
To overlay data on a video, we must mathematically reverse the process of light hitting a sensor.
*   **The Intrinsic Matrix ($K$):** 
    $$ K = \begin{bmatrix} f_x & s & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} $$
    Where $f$ is focal length (pixels), $c$ is the optical center, and $s$ is the skew. 
*   **The Projection:** A 3D point in the world $(X, Y, Z)$ is mapped to a 2D pixel $(u, v)$ via the relationship $\mathbf{x} = K [R|t] \mathbf{X}$. 
*   **The Lesson:** Without $K$, your HUD text will appear to "float" or "slide" as the drone moves because the pixels don't align with the physics of the lens.

#### **2. Radial Distortion & Polynomial Models**
Cheap wide-angle lenses (like our Arducam) create a "Fishbowl" effect.
*   **The Brown-Conrady Model:** We model distortion using a Taylor series expansion of the radius from the optical center. 
*   **Radial Distortion:** $x_{distorted} = x(1 + k_1r^2 + k_2r^4 + k_3r^6)$. 
*   **The Tool:** We use OpenCV to solve for $k_n$ using a checkerboard. If you skip this, your "Virtual Horizon" will only be flat in the exact center of the screen, causing fatal errors in VIO (Module 13).

#### **3. Extrinsics: The SE(3) Transform Tree**
Your Lidar is not at the center of the drone; it might be $3\text{cm}$ offset.
*   **The Rigid Body Transform:** Every sensor reading is a vector in its own coordinate frame. To fuse them, we must map them into the **Body Frame** ($B$) using an element of the **Special Euclidean Group** $SE(3)$, consisting of a rotation $R \in SO(3)$ and a translation $t \in \mathbb{R}^3$.
*   **The Offset Error:** If the drone rotates at $\omega$, an offset sensor $r$ experiences a centripetal acceleration $a = \omega \times (\omega \times r)$. If you don't subtract this, your EKF will think the drone is "jumping" every time it yaws.

**Next Step:** [Phase III: Module 4 Signal Processing](../../Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Lecture.md)

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"We are teaching the machine to see, but first, we must correct its vision. A camera is a collection of lies—radial distortion, rolling shutter, and motion blur. To fly, the drone must look through these lies and find the geometric truth of the room. Today, we bridge the gap between 2D pixels and 3D reality."

### **Deep Research Context: The Rolling Shutter Problem**
In research-grade VIO, we must account for the **Rolling Shutter** of the IMX219 sensor. The camera reads pixels row-by-row. If the drone is moving at $2\text{ m/s}$, the top of the image was captured $33\text{ms}$ before the bottom. This makes straight walls look like they are "leaning." Modern research papers solve this by "Timestamping" each row of pixels and interpolating the drone's position for every line of the image.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Define the components of the Intrinsic Matrix ($K$) and their physical meaning.
- [ ] Explain why "Undistortion" must happen before calculating spatial HUD overlays.
- [ ] Calculate a simple 3D extrinsic transform between two offset sensors.
- [ ] Identify the "Rolling Shutter" effect in high-speed flight footage.

---

## **Further Reading & Bibliography**

### **Calibration & Vision**
*   **Zhang, Z. (2000).** *"A flexible new technique for camera calibration."* IEEE Transactions on Pattern Analysis and Machine Intelligence. (The landmark paper for checkerboard calibration).
*   **Hartley, R., & Zisserman, A. (2003).** *Multiple View Geometry in Computer Vision.* Cambridge University Press. (The definitive textbook).

### **Optics**
*   **Brown, D. C. (1966).** *"Decentering Distortion of Lenses."* Photogrammetric Engineering. (The origin of modern distortion math).

---

[Previous Module](../Module_02_Telemetry_Stack/Module_02_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../../Phase_3_The_Engineer/Module_04_Signal_Processing/Module_04_Lecture.md)