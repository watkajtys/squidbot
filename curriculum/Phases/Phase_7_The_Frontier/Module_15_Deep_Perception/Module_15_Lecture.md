[Previous Module](../Module_14_Swarm_Theory/Module_14_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_15_5_Edge_AI/Module_15_5_Lecture.md)

---

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

### **15.3.1 Just-In-Time Math: The Minecraft Texture (Quantization)**
**"High Res to Pixel Art"**

Your Neural Network is a 4K Photo. It uses 32-bit decimals ($0.123456...$).
Your Pi Zero is a GameBoy. It cannot handle 4K.
*   **Quantization:** We "squash" the 4K photo into an 8-bit Minecraft texture.
*   **The Trick:** We find the Min and Max values of the weights.
    *   $Min (-3.0) \to 0$
    *   $Max (+3.0) \to 255$
*   **The Cost:** You lose a tiny bit of precision ("The texture looks blocky").
*   **The Gain:** It runs 4x faster and uses 4x less RAM.

**AI Prompt:** "Explain 'Post-Training Static Quantization' in PyTorch. How do we map float32 weights to int8 without retraining the model?"

---

## **Check: The Memory**
**Verification Goal:**
Feed the drone a series of images from a flight. Can the algorithm correctly identify when it has returned to the "Home" position using only the camera?

*The academic papers behind this module are listed in the references below.*

---
## **Theoretical Foundations**

### Lecture 15: Deep Perception & Metric Learning

#### **1. Differentiable Spatial Representations**
Traditional maps (Octomaps) are static and discrete. In modern research, we represent the world as a **Differentiable Map** using **3D Gaussian Splatting (3DGS)**.
*   **The Math:** We model the world as a collection of Gaussians $G(x) = \exp(-\frac{1}{2}(x-\mu)^T \Sigma^{-1} (x-\mu))$. 
*   **Gradient Flow:** Because the entire map is a continuous function, we can calculate the gradient of "Collision Risk" at any point. The drone "flows" toward the goal by following the negative gradient of occupancy, allowing for path planning that is infinitely smoother than A*.

#### **2. Metric Learning & The Triplet Loss**
How do we teach a drone to "Recognize" a room without a label? We use **Siamese Networks**.
*   **The Objective:** Learn an embedding function $f(x)$ that maps an image to a point in $\mathbb{R}^{128}$.
*   **Triplet Loss:** $L = \sum [||f(x^a) - f(x^p)||^2 - ||f(x^a) - f(x^n)||^2 + \alpha]_+$.
    *   $x^a$ (Anchor): Current room.
    *   $x^p$ (Positive): Same room, different angle.
    *   $x^n$ (Negative): Different room.
*   **Loop Closure:** By comparing the embedding of the current frame to a database of old frames using **Cosine Similarity**, the drone can "recognize" it has returned home and reset its VIO drift instantly.

#### **3. On-Device Neural Inference (Edge AI)**
A Pi Zero cannot run a 100MB model. We utilize **Model Quantization**.
*   **Quantization:** We map 32-bit floats $[min, max]$ to 8-bit integers $[0, 255]$. 
*   **Latency:** This reduces the computational cost of Matrix Multiplication by $4\text{--}8\text{x}$, allowing for a real-time vision-based "Brain" running on a $\$15$ processor.

**Final Mission:** [The Capstones](../../../../FINAL_CHALLENGE.md)

---

## **Professor's Brief: Speaker Notes**

### **The Narrative Hook**
"A camera is just a light sensor. Deep Perception is the magic that turns 'Light' into 'Meaning.' In this final module, we give the machine a memory of the room. It doesn't just know 'I am 2 meters from a wall'; it knows 'I am in the Kitchen, near the fridge'. We are bridging the final gap between geometric robotics and artificial intelligence."

### **Deep Research Context: The Catastrophic Forgetting Problem**
In PhD research, we care about **Online Learning**. If the drone moves from a bright room to a dark room, its "Recognition" embeddings will fail. Mention that we use **Experience Replay**: we keep a small "Memory Bank" of previous successful recognitions and "Remind" the AI of them during flight to prevent it from forgetting what its home base looks like as the lighting changes.

## **Mastery Checklist**
Before moving to the next module, you should be able to:
- [ ] Explain why 3DGS is superior to discrete Voxel Grids for gradient-based path planning.
- [ ] Define the Anchor, Positive, and Negative terms in Triplet Loss.
- [ ] Describe the Cosine Similarity threshold used for Global Loop Closure.
- [ ] Explain the benefit of Experience Replay for online on-device learning.

---

## **Further Reading & Bibliography**

### **Metric Learning**
*   **Schroff, F., Kalenichenko, D., & Philbin, J. (2015).** *"FaceNet: A unified embedding for face recognition and clustering."* IEEE Conference on Computer Vision and Pattern Recognition (CVPR). (The Triplet Loss paper).

### **Differentiable Mapping**
*   **Lenczner, G., et al. (2024).** *"Differentiable Mapping for Autonomous Navigation."* arXiv preprint. (Cutting-edge research).
*   **Howard, A. G., et al. (2017).** *"MobileNets: Efficient Convolutional Neural Networks for Mobile Vision Applications."* arXiv preprint.

---

[Previous Module](../Module_14_Swarm_Theory/Module_14_Lecture.md) | [Course Map](../../../../COURSE_MAP.md) | [Next Module](../Module_15_5_Edge_AI/Module_15_5_Lecture.md)