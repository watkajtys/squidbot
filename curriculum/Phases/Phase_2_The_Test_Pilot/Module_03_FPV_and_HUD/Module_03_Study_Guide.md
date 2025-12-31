[Return to Course Map](../../../../COURSE_MAP.md)

# Study Guide 3: Computer Vision and Geometry
**Module 3: Sensing the World**

### Critical Takeaways
1.  **The Pinhole Model:** A camera is a mathematical operator that transforms 3D world coordinates $(X, Y, Z)$ into 2D image pixels $(u, v)$. This projection is governed by the Intrinsic Matrix (K), which encapsulates focal length, optical center, and pixel skew.
2.  **Lens Distortion:** Micro-drone lenses, especially fisheye optics, introduce significant Radial and Tangential distortion. To use camera data for navigation (VIO), we must mathematically "rectify" the images using five key distortion coefficients ($k_1, k_2, k_3, p_1, p_2$).
3.  **The Extrinsic Matrix:** This matrix defines the camera's pose (Rotation and Translation) relative to the drone's center of gravity. Even a 1-degree misalignment in your "Extrinsic Calibration" will cause your Visual Odometry to drift by meters over a short flight.

### Mental Models and Field Notes
*   **The Virtual String:** Imagine every pixel on your sensor has an infinite string attached to it, extending out into the world. Every point in 3D space lies on exactly one of these strings. Calibration is simply the process of figuring out the exact 3D angle of every single string.
*   **The Projection "Squish":** When you project 3D to 2D, you lose the "Depth" information (the Z-axis). This is why a single camera cannot tell if a wall is 1 meter away or 10 meters away—it needs "Stereo Vision" or "Motion" to recover that lost dimension.
*   **Frame of Reference:** In computer vision, the "Camera Frame" is often defined with Z pointing *out* of the lens. However, in drone physics, Z usually points *down*. Mastering the rotation matrix that bridges these two frames is the first step to becoming a robotics engineer.

### Frontier Facts and Historical Context
*   **Neural Radiance Fields (NeRFs):** The "Pinhole Model" is being disrupted. New "Neural" cameras don't use matrices; they store the world as a continuous mathematical function. This allows a drone to "generate" a view from a corner it hasn't even visited yet, just by guessing the light distribution in the room.
*   **Event-Based Vision:** Retinal-inspired "Event Cameras" only report when a pixel changes brightness. This allows them to capture motion at the equivalent of 10,000 frames per second with zero motion blur, enabling drones to dodge arrows or fly through spinning fans.
*   **The Lens-less Future:** Researchers at Caltech are building "Flat Cameras" that use an array of light sensors and complex math to reconstruct an image without any lens at all. This would allow drones to have cameras as thin as a piece of paper, covering their entire frame.

---

### The Squid Games: Level 3
**The Horizon Challenge**
Using your AR HUD logic from Module 3, tilt the drone in your hand. The "Virtual Horizon" on your screen must stay perfectly level with the actual floor, regardless of how you rotate the drone.
*   **Win Condition:** A video demonstration showing the AR overlay staying rock-solid while the camera moves. This proves your **Extrinsic Calibration** and **IMU-to-Camera Fusion** are perfectly aligned.

---

### Module 3 Quiz
1.  **Focal Length:** Mathematically, how does increasing the focal length change the relationship between world units (meters) and image units (pixels)?
2.  **Projection:** Why do we divide the $(X, Y)$ coordinates by the depth $(Z)$ during the projection step?
3.  **Calibration:** Why is a high-contrast chessboard used for calibration? What is a "Saddle Point," and why is it easier for a computer to find than a simple dot?
4.  **Homogeneous Coordinates:** Why do we use 4x4 matrices for 3D transformations instead of 3x3 matrices? What does that "extra" 4th dimension allow us to do?

---
*Reference: Lecture 3 (Computer Vision) in docs/LECTURES.md*