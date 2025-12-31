[Return to Course Map](../../../../COURSE_MAP.md)

# Study Guide 15.5: Edge AI and Model Optimization
**Module 15.5: The Frontier (Efficiency Engineering and On-Device AI)**

### Critical Takeaways
1.  **Latency is the Killer:** In high-speed robotics, a slow AI is a useless AI. If your perception model runs at 2Hz (500ms latency), your drone will have traveled several meters before it "sees" an obstacle. We optimize for **Frames Per Second (FPS)** as much as we optimize for Accuracy.
2.  **Quantization (INT8) is Mandatory:** The Raspberry Pi Zero 2 W does not have the "Horsepower" to perform heavy FP32 (32-bit floating point) math for thousands of neurons. By converting the model to INT8, we trade a tiny amount of precision for a massive gain in speed and a 75% reduction in memory footprint.
3.  **Efficiency Architecture:** You don't just "Optimize" a bad model; you start with a good one. Architectures like **MobileNetV2/V3** use Depthwise Separable Convolutions to reduce the number of mathematical operations (FLOPs) by 8-9x compared to standard CNNs.

### Mental Models and Field Notes
*   **The von Neumann Bottleneck:** In embedded systems, the bottleneck is often the "Road" (the memory bus), not the "Factory" (the CPU). If your AI model is too big to fit in the CPU's cache, the processor spends most of its time waiting for weights to arrive from RAM.
*   **The Pareto Frontier:** There is no "Perfect Model." There is only a trade-off curve between Accuracy and Latency. Your job as an engineer is to find the "Sweet Spot" on this curve where the AI is smart enough to detect a target but fast enough to react to it.
*   **Dead Wood (Pruning):** Not all neurons are equal. In a trained network, many weights are near zero. Pruning is the act of "Cutting" these useless connections to make the model leaner and faster without losing intelligence.

### Frontier Facts and Historical Context
*   **Neuromorphic Sensors:** The future of Edge AI isn't just better software; it's better hardware. "Event Cameras" (like the DVS) only send data when a pixel changes intensity. This allows for microsecond-level reaction times at 1/100th of the power of a standard camera.
*   **Neural Architecture Search (NAS):** Instead of humans designing models, we use AI to design the AI. Google's **MnasNet** was specifically designed by an algorithm to find the fastest possible model for a specific mobile processor.
*   **INT8 Integer-Only Inference:** Some advanced microcontrollers (like the ESP32-S3 or the ARM Cortex-M7) don't even have a hardware Floating Point Unit (FPU). For these chips, INT8 quantization isn't just an optimization—it's the only way to run AI at all.

---

### The Squid Games: Level 15.5
**The Silicon Challenge**
Deploy two versions of your target detector to the Pi Zero.
1.  **Version A:** The raw Keras/PyTorch model (FP32).
2.  **Version B:** The TFLite-Optimized model (INT8).
*   **The Goal:** Run both simultaneously (or sequentially) and measure the `inference_time` per frame.
*   **Win Condition:** Seeing Version B run at least 3x faster than Version A while maintaining the same detection success on the "Survivor" test set.

---

### Module 15.5 Quiz
1.  **Quantization:** Explain the formula $q = \text{round}(r/S + Z)$. What do the $S$ (Scale) and $Z$ (Zero-point) variables represent?
2.  **MobileNet:** How does a "Depthwise Separable Convolution" differ from a standard convolution? Why does it save so many FLOPs?
3.  **Memory:** Why does a 100MB model run slower than a 10MB model even if they have the same number of layers?
4.  **Trade-offs:** If your drone is flying in a wide-open field, would you prioritize Accuracy or Latency? What if it is flying through a dense forest?

---
*Reference: Lecture 15.5 (Edge AI) in curriculum/Phases/Phase_7_The_Frontier/Module_15_5_Edge_AI/Module_15_5_Lecture.md*
