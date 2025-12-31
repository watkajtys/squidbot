# Module 15: Deep Perception (Advanced)
**"The Memory of the Machine."**

*(Status: Advanced Research Module - Lab Pending)*

In this final module, we move beyond geometry. We explore how Neural Networks can give the drone a sense of "Recognition." This allows for **Global Localization**—knowing where you are because you "remember" the room.

---

## **15.1 CNNs for Visual Landmarks**

### **Objective**
Extract robust features from images that are invariant to lighting and angle.

### **Theory**
*   **Convolutional Neural Networks (CNNs):** Instead of looking at pixels, the drone looks for "edges," "textures," and "shapes."
*   **Metric Learning:** We don't want to classify "Cat vs. Dog." We want to know if "Image A" and "Image B" are the same physical location.

---

## **15.2 Siamese Networks & Triplet Loss**

### **The Problem**
If the drone sees a corner of a room, and then sees it again 5 minutes later, how does it know it has "Closed the Loop"? 

### **The Solution**
We train a Siamese Network to produce a **Feature Vector** (a list of 128 numbers).
*   If two images are from the same spot, their vectors will be "close" in space.
*   If they are from different rooms, their vectors will be "far" apart.

---

## **15.3 Deployment (On-Device Inference)**

### **The Challenge**
Neural Networks are heavy. The Raspberry Pi Zero 2 W is small.
*   **Quantization:** Converting 32-bit weights to 8-bit to save RAM and CPU.
*   **TFLite:** Using the TensorFlow Lite runtime for efficient execution.

---

## **Check: The Memory**
**Verification Goal:**
Feed the drone a series of images from a flight. Can the algorithm correctly identify when it has returned to the "Home" position using only the camera?

*Refer to Lecture 15 in LECTURES.md for the academic papers behind this module.*
