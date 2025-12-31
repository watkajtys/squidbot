# Study Guide 15: Deep Perception
**Module 15: The Frontier (AI and Neural Navigation)**

### Critical Takeaways
1.  **Metric Learning:** Traditional AI classifies images (e.g., "This is a wall"). Metric learning teaches the robot to measure the "distance" between scenes. This allows for **Loop Closure**—recognizing a previously visited room even if the furniture has been rearranged.
2.  **Embeddings:** A high-dimensional image (millions of pixels) is compressed into a low-dimensional "embedding" vector (e.g., 128 numbers). This vector captures the semantic essence of a scene while ignoring minor "noise" like shadows.
3.  **Siamese Networks:** A neural network architecture with two identical branches that share the same weights. By processing two different images simultaneously, the network learns to output vectors that are "close" if the images are of the same location.
4.  **Inference on the Edge:** Running deep neural networks on a drone requires techniques like **Quantization** (converting 32-bit floats to 8-bit integers) and **Pruning** to fit large models onto the Raspberry Pi Zero's limited CPU.

### The Evolution of Perception
*   **The Foundation (Hand-Crafted Features):** Early vision relied on human-designed math to find "interest points" (like the KLT tracker in Module 13). These are fast but fail when the lighting changes or the scene is repetitive.
*   **The Industry Standard (Deep Embeddings):** This is the focus of `lab_15_siamese_perception.py`. We use CNNs to "learn" what makes a room unique. This is much more robust but still treats the world as a 2D image.
*   **The Frontier (3D Gaussian Splatting):** The newest standard for spatial awareness. 3DGS represents the world as millions of differentiable 3D ellipsoids. This allows the drone to not just "recognize" a room, but to mathematically reconstruct its 3D geometry and lighting, enabling sub-centimeter navigation without any hand-crafted landmarks.

### Mental Models and Field Notes
*   **The Robot's Memory:** When a drone performs loop closure, it isn't just "recognizing" a room. It is effectively reconciling its two versions of reality: its **Internal Guess** (where it thinks it is based on drift) and the **Visual Truth** (where it actually is). That "Aha!" moment allows the drone to snap its entire history back into alignment.
*   **Feature over Pixel:** Why do we use embeddings instead of comparing pixels? Because a single shadow moving across the floor can change thousands of pixels, but it doesn't change the *meaning* of the room.
*   **The Search for Saliency:** Not all pixels are equal. A blank white wall tells the drone nothing about where it is. A unique corner or a specific poster is "Salient." Deep Perception is the art of teaching a robot to ignore the boring and focus on the unique.

### Frontier Facts and Historical Context
*   **The Power Efficiency Frontier (Spiking Neural Networks):** SNNs work like the human brain—they only "fire" a signal when a threshold is met. This allows micro-drones the size of an insect to navigate for hours on a tiny battery that would normally last only seconds.
*   **3D Gaussian Splatting (3DGS):** This represents the world as millions of tiny, transparent ellipsoids. Because Gaussians are differentiable, a drone can "render" a predicted view and compare it to its actual camera feed to calculate its position with sub-centimeter accuracy—even in featureless environments.
*   **The Ethics of Autonomy:** If an RL policy decides to crash into a wall, *why* did it do it? In the field, we use "Policy Saliency" maps to see exactly what parts of the image the neural network was "looking at" when it made its decision.

---

### The Squid Games: Level 15
**The Hallucination Challenge**
Show the camera two different rooms that have similar white walls and generic furniture. Can the perception engine tell them apart?
*   **The Goal:** Adjust your **Similarity Threshold** and **Feature Descriptor** until the engine stops "hallucinating" that the two rooms are the same.
*   **Win Condition:** Zero false-positive loop closures over a 5-minute flight through multiple rooms.

---

### Module 15 Quiz
1.  **Metric Learning:** Why is "Cosine Similarity" often used instead of "Euclidean Distance" when comparing image embeddings?
2.  **Triplet Loss:** Explain the role of the "Anchor," "Positive," and "Negative" samples during the training of a Siamese network.
3.  **Loop Closure Drift:** How does recognizing a "Loop Closure" physically help correct the accumulated error in a drone's state estimate?
4.  **Architecture Selection:** Why do we prefer architectures like **MobileNet** or **SqueezeNet** for on-board perception instead of deeper models like ResNet-101?

---
*Reference: Module 15 in docs/Module_15_Deep_Perception.md*